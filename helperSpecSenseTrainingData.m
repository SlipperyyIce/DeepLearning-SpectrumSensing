function helperSpecSenseTrainingData(numFrames,imageSize,trainDir,numSF,outFs)
%helperSpecSenseTrainingData Training data for spectrum sensing
%   helperSpecSenseTrainingData(N,S,DIR,NSF,FS) generates training data for
%   the Spectrum Sensing with Deep Learning to Identify 5G and LTE Signals
%   example. Traning data is the image of the spectrogram of baseband
%   frames together with the pixel labels. The function generates N groups
%   of frames where each group has an LTE only frame, 5G only frame and a
%   frame that contains both 5G and LTE signals. Single signal images are
%   saved in DIR. Combined signal images are saved to DIR/LTE_NR. The
%   images have a size of S, which is a 1x2 vector or integers. The sample
%   rate of the signals is FS. Each frame is NSF subframes long.
%
%   5G NR signals are based on SISO configuration for frequency range 1
%   (FR1). LTE signals are based on SISO configuration with frequency
%   division duplexing (FDD). 
%
%   See also helperSpecSenseNRSignal, helperSpecSenseLTESignal.

%   Copyright 2021 The MathWorks, Inc.

combinedDir = fullfile(trainDir,'LTE_NR');
if ~exist(combinedDir,'dir')
  mkdir(combinedDir)
end
combinedDir = fullfile(trainDir,'BT_WLAN');
if ~exist(combinedDir,'dir')
  mkdir(combinedDir)
end

files = dir(fullfile(combinedDir,'*.png'));

maxFrameNum = 2;
for p=1:length(files)
  frameNum = str2double(strtok(strtok(files(p).name, 'LTE_NR_frame_'), '.'));
  if frameNum > maxFrameNum
    maxFrameNum = frameNum;
  end
end

% 5G Parameters
SCSVec = [15 30];
BandwidthVec = [10:5:30 40 50]; % [60 80 90 100]
maxTimeShift = numSF;  % Time shift in milliseconds
SSBPeriodVec = 20; %[5 10 20 40 80 160] 20 is most frequenctly found OTA

% LTE Parameters
RCVec = {'R.2','R.6','R.8','R.9'};
TrBlkOffVec = {1,2,3,4,5,6,7,8};

% Channel Parameters
SNRdBVec = {40, 50, 100};   % dB
SNRdBVec2 = {40, 50,60}; % db
SNRdBVec3 = {20,30,40,}; % db
DopplerVec = {0, 10, 500};
CenterFrequencyVec = 4e9;

if exist("gcp","file")
  pool = gcp('nocreate');
  if isempty(pool)
    pool = parpool;
  end
  numWorkers = pool.NumWorkers;
else
  numWorkers = 1;
end
numFramesPerWorker = ceil(numFrames / numWorkers);

tStart = tic;
parfor parIdx=1:numWorkers
  frameIdx = maxFrameNum - numFramesPerWorker*(numWorkers-1) - numWorkers;
  if frameIdx < 0
    frameIdx = 0;
  end
  while frameIdx < numFramesPerWorker
    % Generate 5G signal
    scs = SCSVec(randi([1 length(SCSVec)])); %#ok<*PFBNS> 
    nrChBW = BandwidthVec(randi([1 length(BandwidthVec)]));
    timeShift = rand()*maxTimeShift;
    SSBPeriod = SSBPeriodVec(randi([1 length(SSBPeriodVec)]));
    [txWave5G, waveinfo5G] = helperSpecSenseNRSignal(scs,nrChBW,SSBPeriod,timeShift,numSF,outFs);
    
    % Generate LTE signal
    RC = RCVec{randi([1 length(RCVec)])};
    timeShift = rand()*maxTimeShift;
    TrBlkOff = TrBlkOffVec{randi([1 length(TrBlkOffVec)])};
    [txWaveLTE, waveinfoLTE] = helperSpecSenseLTESignal(RC,timeShift,TrBlkOff,numSF,outFs);

    % Generate BT signal
    timeShift = rand()*maxTimeShift;
    [txWaveBT, waveinfoBT] = helperSpecSenseBTSignal(timeShift);
    
    % Generate WLANsignal
    timeShift = rand()*maxTimeShift;
    [txWaveWLAN, waveinfoWLAN,iwnConfig] = helperSpecSenseWLANSignal(timeShift);
      
    % Generate Noise
    rng('shuffle');
    txWaveNoise = wgn(256*256,1,0) * 0.1;
    paramsL = struct();
    paramsL.SCS = scs;
    
    %saveSpectrogramImage(txWaveNoise,40e6,trainDir,'Noise',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
   


    % Decide on channel parameters
    SNRdB = SNRdBVec{randi([1 length(SNRdBVec)])};
    Doppler = DopplerVec{randi([1 length(DopplerVec)])};
    Fc = CenterFrequencyVec(randi([1 length(CenterFrequencyVec)]));
    
    % Save channel impared 5G signal spectrogram and pixel labels
    sr = waveinfo5G.ResourceGrids.Info.SampleRate;
    rxWave5G = multipathChannel5G(txWave5G,sr,Doppler,Fc);
    [rxWave5G,freqOff] = shiftInFrequency(rxWave5G,waveinfo5G.Bandwidth,sr,imageSize(2));
    rxWave5G = awgn(rxWave5G,SNRdB);

    params5G = struct();
    params5G.SCS = scs;
    params5G.BW = nrChBW;
    params5G.SSBPeriod = SSBPeriod;
    params5G.SNRdB = SNRdB;
    params5G.Doppler = Doppler;
    params5G.Info = waveinfo5G;
    saveSpectrogramImage(rxWave5G,sr,trainDir,'NR',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    freqPos = freqOff' + [-waveinfo5G.Bandwidth/2 +waveinfo5G.Bandwidth/2]';
    
    savePixelLabelImage({[]}, freqPos, {'NR'}, {'Noise','NR','LTE','BT','WLAN'}, sr, params5G, trainDir, imageSize, frameIdx+(numFramesPerWorker*(parIdx-1)),0)

    % Save channel impared LTE signal spectrogram and pixel labels
    rxWaveLTE = multipathChannelLTE(txWaveLTE,waveinfoLTE.SampleRate,Doppler);
    [rxWaveLTE,freqOff] = shiftInFrequency(rxWaveLTE, ...
      waveinfoLTE.Bandwidth, ...
      waveinfoLTE.SampleRate, imageSize(2));
    rxWaveLTE = awgn(rxWaveLTE,SNRdB);

    

    paramsLTE = struct();
    paramsLTE.RC = RC;
    paramsLTE.SNRdB = SNRdB;
    paramsLTE.TrBlkOff = TrBlkOff;
    paramsLTE.Doppler = Doppler;
    paramsLTE.Info = waveinfoLTE;
    saveSpectrogramImage(rxWaveLTE,waveinfoLTE.SampleRate,trainDir,'LTE',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    freqPos = freqOff' + [-waveinfoLTE.Bandwidth/2 +waveinfoLTE.Bandwidth/2]';
    
    savePixelLabelImage({[]}, freqPos, {'LTE'}, {'Noise','NR','LTE','BT','WLAN'}, waveinfoLTE.SampleRate, paramsLTE, trainDir, imageSize, frameIdx+(numFramesPerWorker*(parIdx-1)),0)


    % Save BT signal spectrogram and pixel labels
    rng('shuffle');
    SNRdB = SNRdBVec2{randi([1 length(SNRdBVec)])};
    
    rxWaveBT = awgn(txWaveBT,SNRdB);
    sr = waveinfoBT.SampleRate;
    %look into multipathing?????
    
    [rxWaveBT,freqOff] = shiftInFrequency(rxWaveBT,waveinfoBT.Bandwidth,sr,imageSize(2));
    
    paramsBT = struct();
    paramsBT.SNRdB = SNRdB;
    paramsBT.Info = waveinfoBT;
    saveSpectrogramImage(rxWaveBT,sr,trainDir,'BT',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    freqPos = freqOff' + [-waveinfoBT.Bandwidth/2 +waveinfoBT.Bandwidth/2]';

    savePixelLabelImage({[]}, freqPos, {'BT'}, {'Noise','NR','LTE','BT','WLAN'}, waveinfoBT.SampleRate, paramsBT, trainDir, imageSize, frameIdx+(numFramesPerWorker*(parIdx-1)),0) 
   
    % Save WLAN signal spectrogram and pixel labels
    rng('shuffle');
    SNRdB = SNRdBVec2{randi([1 length(SNRdBVec)])};
    rxWaveWLAN = awgn(txWaveWLAN,SNRdB);
    sr = waveinfoWLAN.SampleRate;
    [rxWaveWLAN,freqOff] = shiftInFrequency(rxWaveWLAN,waveinfoWLAN.Bandwidth,sr,imageSize(2));
    

    paramsWLAN = struct();
    paramsWLAN.SNRdB = SNRdB;
    paramsWLAN.Info = waveinfoBT;
    saveSpectrogramImage(rxWaveWLAN,sr,trainDir,'WLAN',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    freqPos = freqOff' + [-waveinfoWLAN.Bandwidth/2 +waveinfoWLAN.Bandwidth/2]';

    savePixelLabelImage({[]}, freqPos, {'WLAN'}, {'Noise','NR','LTE','BT','WLAN'}, waveinfoWLAN.SampleRate, paramsWLAN, trainDir, imageSize, frameIdx+(numFramesPerWorker*(parIdx-1)),0) 
   

    % Save combined image
    assert(waveinfo5G.ResourceGrids.Info.SampleRate == waveinfoLTE.SampleRate)

    sr = waveinfoLTE.SampleRate;
    comb = comm.MultibandCombiner("InputSampleRate",sr, ...
      "OutputSampleRateSource","Property",...
      "OutputSampleRate",sr);
    % Decide on the frequency space between LTE and 5G
    maxFreqSpace = (sr - waveinfo5G.Bandwidth - waveinfoLTE.Bandwidth);
    if maxFreqSpace < 0
      continue
    end
    freqSpace = round(rand()*maxFreqSpace/1e6)*1e6;
    freqPerPixel = sr / imageSize(2);
    maxStartFreq = sr - (waveinfo5G.Bandwidth + waveinfoLTE.Bandwidth + freqSpace) - freqPerPixel;

    % Decide if 5G or LTE is on the left
    LTEFirst = randi([0 1]);
    if LTEFirst
      combIn = [txWaveLTE, txWave5G];
      labels = {'LTE','NR'};
      startFreq = round(rand()*maxStartFreq/1e6)*1e6 - sr/2 + waveinfoLTE.Bandwidth/2;
      bwMatrix = [-waveinfoLTE.Bandwidth/2 +waveinfoLTE.Bandwidth/2; -waveinfo5G.Bandwidth/2 +waveinfo5G.Bandwidth/2]';
    else
      combIn = [txWave5G txWaveLTE];
      labels = {'NR','LTE'};
      startFreq = round(rand()*maxStartFreq/1e6)*1e6 - sr/2 + waveinfo5G.Bandwidth/2;
      bwMatrix = [-waveinfo5G.Bandwidth/2 +waveinfo5G.Bandwidth/2; -waveinfoLTE.Bandwidth/2 +waveinfoLTE.Bandwidth/2]';
    end
    comb.FrequencyOffsets = [startFreq startFreq+waveinfoLTE.Bandwidth/2 + freqSpace + waveinfo5G.Bandwidth/2];
    txWave = comb(combIn);

    % Pass through channel
    rxWaveChan = multipathChannel5G(txWave, sr, Doppler, Fc);

    % Add noise
    rxWave = awgn(rxWaveChan,SNRdB);

    % Create spectrogram image
    paramsComb = struct();
    paramsComb.SCS = scs;
    paramsComb.BW = nrChBW;
    paramsComb.SNRdB = SNRdB;
    paramsComb.Doppler = Doppler;
    paramsComb.RC = RC;
    paramsComb.SNRdB = SNRdB;
    paramsComb.TrBlkOff = TrBlkOff;
    paramsComb.Doppler = Doppler;
    saveSpectrogramImage(rxWave,sr,fullfile(trainDir,'LTE_NR'),...
      'LTE_NR',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    freqPos = comb.FrequencyOffsets + bwMatrix;
    savePixelLabelImage({[],[]}, freqPos, labels, {'Noise','NR','LTE','BT','WLAN'}, ...
      sr, paramsComb, fullfile(trainDir,'LTE_NR'), imageSize, ...
      frameIdx+(numFramesPerWorker*(parIdx-1)),0)

    %Generate Combined WLAN + BT
    rng('shuffle');
    [txWave, waveinfoBT_WLAN,~] = helperSpecSenseBT_WLANSignal(imageSize(2));
    % Add noise
    rng('shuffle');
    SNRdB = SNRdBVec3{randi([1 length(SNRdBVec3)])};
    rxWave = awgn(txWave,SNRdB);
    % Create spectrogram image
    paramsComb2 = struct();
    paramsComb2.BW = waveinfoBT_WLAN.Bandwidth;
    paramsComb2.SNRdB = SNRdB;

    saveSpectrogramImage(rxWave,sr,fullfile(trainDir,'BT_WLAN'),...
      'BT_WLAN',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));

    savePixelLabelImage({[],[]}, freqPos, {'BT','WLAN'}, {'Noise','NR','LTE','BT','WLAN'}, ...
      waveinfoBT_WLAN.SampleRate, paramsComb2, fullfile(trainDir,'BT_WLAN'), imageSize, ...
      frameIdx+(numFramesPerWorker*(parIdx-1)),0)


    %Generate Combined BT + BT
    rng('shuffle');
    timeShift = rand()*maxTimeShift;
    [txWave, waveinfoDoubleBT,~] = helperSpecSenseDoubleBTSignal(imageSize(2),timeShift);
    % Add noise
    rng('shuffle');
    SNRdB = SNRdBVec3{randi([1 length(SNRdBVec3)])};
    rxWave = awgn(txWave,SNRdB);
    % Create spectrogram image
    paramsComb3 = struct();
    paramsComb3.BW = waveinfoDoubleBT.Bandwidth;
    paramsComb3.SNRdB = SNRdB;

    saveSpectrogramImage(rxWave,sr,trainDir,...
      'BT_BT',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));

    savePixelLabelImage({[],[]}, freqPos, {'BT','BT'}, {'Noise','NR','LTE','BT','WLAN'}, ...
      waveinfoDoubleBT.SampleRate, paramsComb3, trainDir, imageSize, ...
      frameIdx+(numFramesPerWorker*(parIdx-1)),0)

    

    %Generate Random numbers of BT signals
    rng('shuffle');
    [txWave, waveinfoMultiBT,~,multi_label,timepos] = helperSpecSenseMultiBTSignal(imageSize(2));
    % Add noise
    rng('shuffle');
    SNRdB = SNRdBVec3{randi([1 length(SNRdBVec3)])};
    rxWave = awgn(txWave,SNRdB);
    % Create spectrogram image
    paramsComb3 = struct();
    paramsComb3.BW = waveinfoMultiBT.Bandwidth;
    paramsComb3.SNRdB = SNRdB;

    saveSpectrogramImage(rxWave,sr,trainDir,...
      'BTMULTI',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    
    savePixelLabelImage(timepos, waveinfoMultiBT.freqPos, multi_label, {'Noise','NR','LTE','BT','WLAN'}, ...
      waveinfoMultiBT.SampleRate, paramsComb3, trainDir, imageSize, ...
      frameIdx+(numFramesPerWorker*(parIdx-1)),1)

    %Generate Rnadom numbers of WLAN + BT signals
    rng('shuffle');
    [txWave, waveinfoMultiWLAN,~,multi_label2,timepos2] = helperSpecSenseMultiWLANSignal(imageSize(2));
    % Add noise
    rng('shuffle');
    SNRdB = SNRdBVec3{randi([1 length(SNRdBVec3)])};
    rxWave = awgn(txWave,SNRdB);

    % Create spectrogram image
    paramsComb4 = struct();
    paramsComb4.BW = waveinfoMultiWLAN.Bandwidth;
    paramsComb4.SNRdB = SNRdB;

    saveSpectrogramImage(rxWave,sr,trainDir,...
      'WLANMULTI',imageSize,frameIdx+(numFramesPerWorker*(parIdx-1)));
    
    %savePixelLabelImage(timepos2, waveinfoMultiWLAN.freqPos, multi_label2, {'Noise','NR','LTE','BT','WLAN'}, ...
    %  waveinfoMultiWLAN.SampleRate, paramsComb4, trainDir, imageSize, ...
    %  frameIdx+(numFramesPerWorker*(parIdx-1)),2)

    






    frameIdx = frameIdx + 1;
    if mod(frameIdx,10) == 0
      elapsedTime = seconds(toc(tStart));
      elapsedTime.Format = "hh:mm:ss";
      disp(string(elapsedTime) + ": Worker " + parIdx + ...
        " generated "  + frameIdx + " frames")
    end
  end
  elapsedTime = seconds(toc(tStart));
  elapsedTime.Format = "hh:mm:ss";
  disp(string(elapsedTime) + ": Worker " + parIdx + ...
    " generated "  + frameIdx + " frames")
end
end

% Helper Functions
function [y,freqOff] = shiftInFrequency(x, bw, sr, numFreqPixels)
freqOffset = comm.PhaseFrequencyOffset(...
  'SampleRate',sr);

maxFreqShift = (sr-bw) / 2 - sr/numFreqPixels;
freqOff = (2*rand()-1)*maxFreqShift;
freqOffset.FrequencyOffset = freqOff;
y = freqOffset(x);
end

function y = multipathChannel5G(x, sr, doppler, fc)
% Pass through channel
chan = nrCDLChannel('DelayProfile','Custom',... % one path with
  'SampleRate',sr,...
  'MaximumDopplerShift',doppler,...
  'CarrierFrequency',fc,...
  'Seed', randi(10e3,1,1)); % Random seed to create varying doppler
chan.TransmitAntennaArray.Size = [1 1 1 1 1];
chan.TransmitAntennaArray.Element = 'isotropic';
chan.ReceiveAntennaArray.Size = [1 1 1 1 1];
y = chan(x);
end

function y = multipathChannelLTE(x, sr, doppler)
chan.DelayProfile = 'Custom';
chan.AveragePathGaindB = 0; % single path (LoS) with 0 dB gain and no delay
chan.PathDelays = 0;
chan.NRxAnts = 1;
chan.MIMOCorrelation = 'Low';
chan.Seed = randi(10e3,1,1); % Random seed to create varying doppler
chan.InitPhase = 'Random';
chan.NTerms = 16;
chan.InitTime = 0;
chan.SamplingRate = sr;
chan.DopplerFreq = doppler;

y = lteFadingChannel(chan,x);
end

function saveSpectrogramImage(rxWave,sr,folder,label,imageSize, idx)
Nfft = 4096;

rxSpectrogram = helperSpecSenseSpectrogramImage(rxWave,Nfft,sr,imageSize);

% Create file name
fname = fullfile(folder, [label '_frame_' strrep(num2str(idx),' ','')]);
fname = fname + ".png";
imwrite(rxSpectrogram, fname)
end

function savePixelLabelImage(timePos, freqPos, label, pixelClassNames, sr, params, folder, imSize, idx, rngNoSignal)
data = uint8(zeros(imSize));
for p=1:length(label)
  pixelValue = floor((find(strcmp(label{p}, pixelClassNames))-1)*255/(numel(pixelClassNames)-1));
  freqPerPixel = sr / imSize(2);
  try
    
    freqPixels = floor((sr/2 + freqPos(:,p)) / freqPerPixel) + 1; %FIX BRO
  catch ME
    disp(freqPos);
    error('Stopping script due to error: %s')
   
  end
  
  if isempty(timePos{p})
   timePixels = 1:imSize(1);
  end
  data(timePixels,freqPixels(1):freqPixels(2)) = uint8(pixelValue);

end

% Create file name
if numel(label) == 1
  lbl = label{1};
else
  lbl = 'LTE_NR';

  if strcmp(label{1}, "BT")
      lbl = 'BT_WLAN';
  end
  if strcmp(label{1}, "BT") && strcmp(label{2}, "BT") 
      lbl = 'BT_BT';
  end

  if rngNoSignal == 1
    lbl = 'BTMULTI';
  end
  if rngNoSignal == 2
    lbl = 'WLANMULTI';
  end
end
fname = fullfile(folder, [lbl '_frame_' strrep(num2str(idx),' ','')]);
fnameLabels = fname + ".hdf";
imwrite(data,fnameLabels,'hdf');

fnameParams = fname + ".mat";
save(fnameParams, "params")
end
