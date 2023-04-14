function [txWaveInt,waveInfo,iwnConfig,label] = helperSpecSenseMultiBTSignal(numFreqPixels)
%numFreqPixels = 256;
rng('shuffle');
BtType = {'LE1M',"LE2M","LE500K","LE125K"};
rand_index = randi(numel(BtType)); % generate random index
awnSignalType = BtType{rand_index}; % select random string
awnTxPosition =  [0,0,0];    % In meters
awnRxPosition =  [0,0,0];    % In meters
awnTxPower = 30;       % In dBm
awnPacket = "Disabled";
awnFrequencyHopping = "Off";
awnFrequency = 2440*1e6; % In Hz

iwnType = ["LE1M","LE2M","LE500K","LE125K","BR","EDR2M","EDR3M"];
rand_rge = randi([1, 7]);
for i = 1:rand_rge
    rng('shuffle');
    rand_index = randi(numel(iwnType)); 
    iwn(i).SignalType  = iwnType{rand_index}; 
    iwn(i).TxPosition = [0,0,0];      % In meters
    iwn(i).Frequency = 2420e6;         % In Hz
    iwn(i).TxPower = 30;               % In dBm
    iwn(i).CollisionProbability = 0.2;   % Probability of collision in time, must be between [0,1]

    
    switch iwn(i).SignalType
        case {'802.11b/g with 22 MHz Bandwidth'}
            iwn(i).Bandwidth = 22e6;
        case {'802.11g with 20 MHz Bandwidth',...
            '802.11n with 20 MHz Bandwidth','802.11ax with 20 MHz Bandwidth', 'WLANHESUBandwidth20.bb'}
            iwn(i).Bandwidth = 20e6;
        case {'802.11n with 40 MHz Bandwidth','802.11ax with 40 MHz Bandwidth'}
            iwn(i).Bandwidth= 40e6;
        case {'LE1M','LE2M','LE500K','LE125K'}
            iwn(i).Bandwidth = 2e6;
        case {'BR','EDR2M','EDR3M'}
            iwn(i).Bandwidth = 1e6;
        otherwise
            disp(iwn(1).SignalType);
    end
end

environment = "Outdoor";
EbNo = 10;         % In dB
sampleRate = 80e6; % In Hz
numPackets = 1;

rng default;

phyFactor = 1+strcmp(awnSignalType,"LE2M");
sps = sampleRate/(1e6*phyFactor);                       % Samples per symbol
if sps > 8                                              % Decimation factor for the receiver filter
    decimationFactor = gcd(sps,8);
else
    decimationFactor = 1;
end

if any(strcmp(awnSignalType,["LE1M","LE2M","LE500K","LE125K"]))
    payloadLength = 100;                                % Length of the payload in bytes
    accessAddress = "01234567";                         % Access address
    accessAddBits = int2bit(hex2dec(accessAddress),32,false);

    % Derive channel index based on the AWN frequency
    channelIndexArray = [37 0:10 38 11:36 39];
    awnBandwidth = 2e6;
    channelIndex = channelIndexArray((awnFrequency-2402e6)/awnBandwidth+1);
    
    % Configure the receiver parameters in a structure
    rxCfg = struct(Mode=awnSignalType,SamplesPerSymbol=sps/decimationFactor,ChannelIndex=channelIndex, ...
        DFPacketType=awnPacket,AccessAddress=accessAddBits);
    rxCfg.CoarseFreqCompensator = comm.CoarseFrequencyCompensator(Modulation="OQPSK", ...
                                             SampleRate=sampleRate/decimationFactor, ...
                                             SamplesPerSymbol=2*rxCfg.SamplesPerSymbol, ...
                                             FrequencyResolution=100);
    rxCfg.PreambleDetector = comm.PreambleDetector(Detections="First");
else
    % Create and configure Bluetooth waveform generation parameters
    awnWaveformConfig = bluetoothWaveformConfig(Mode=awnSignalType,PacketType=awnPacket, ...
                            SamplesPerSymbol=sps);
    if strcmp(awnPacket,"DM1")
        awnWaveformConfig.PayloadLength = 17;            % Maximum length of DM1 packets in bytes
    end
    payloadLength = getPayloadLength(awnWaveformConfig); % Length of the payload

    % Get the receiver configuration parameters
    rxCfg = getPhyConfigProperties(awnWaveformConfig);
    rxCfg.SamplesPerSymbol = sps/decimationFactor;
end

% Estimate distance between AWN transmitter and AWN receiver
distanceAWNTxRx = sqrt(sum((awnTxPosition-awnRxPosition).^2));
[awnPathloss,pathlossdB] = helperBluetoothEstimatePathLoss(environment,distanceAWNTxRx);

iwnConfig = helperIWNConfig(IWN=iwn,SampleRate=sampleRate,Environment=environment);
iwnWaveform = generateIWNWaveform(iwnConfig);
[iwnWaveformPL,iwnPathloss] = applyPathloss(iwnConfig,iwnWaveform,awnRxPosition);

if strcmp(awnFrequencyHopping,"On")
    if any(strcmp(awnSignalType,["LE1M","LE2M","LE500K","LE125K"]))
        frequencyHop = bleChannelSelection;   % Bluetooth LE channel index System object™
        numBTChannels = 37;                   % Number of Bluetooth LE channels
        minChannels = 2;                      % Minimum number of channels to classify
    else
        frequencyHop = bluetoothFrequencyHop; % Bluetooth BR/EDR channel index object
        frequencyHop.SequenceType = "Connection Adaptive";
        numBTChannels = 79;                   % Number of Bluetooth BR/EDR channels
        minChannels = 20;                     % Minimum number of channels to classify
        inputClock = 0;
        numSlots = 1*(any(strcmp(awnPacket,["ID","NULL","POLL","FHS","HV1","HV2", ...
                    "HV3","DV","EV3","DM1","DH1","AUX1","2-DH1","3-DH1","2-EV3","3-EV3"])))...
                    +(3*any(strcmp(awnPacket,["EV4","EV5","DM3","DH3","2-EV5","3-EV5","2-DH3", ...
                    "3-DH3"])))+ (5*any(strcmp(awnPacket,["DM5","DH5","2-DH5","3-DH5"])));
        slotValue = numSlots*2;
        clockTicks = slotValue*2;             % Clock ticks (one slot is two clock ticks)
    end
end

    if any(strcmp(awnSignalType,["EDR2M","EDR3M"]))
        rolloff = 0.4;
        span = 8;
        filterCoeff = rcosdesign(rolloff,span,sps,"sqrt");
    else
        N = 200;                                   % Order
        Fc = 1.5e6/(1+strcmp(awnSignalType,"BR")); % Cutoff frequency
        flag = "scale";                            % Sampling flag
        alpha = 3;                                 % Window parameter
        
        % Create the window vector for the design algorithm
        win = gausswin(N+1,alpha);
        
        % Calculate the coefficients using the FIR1 function
        filterCoeff = fir1(N,Fc/(sampleRate/2),"low",win,flag);
    end
    firdec = dsp.FIRDecimator(decimationFactor,filterCoeff);

codeRate = 1*any(strcmp(awnSignalType,["LE1M","LE2M"]))+1/2*strcmp(awnSignalType,"LE500K")+1/8*strcmp(awnSignalType,"LE125K")+...
           any(strcmp(awnSignalType,["BR","EDR2M","EDR3M"]))*(1-2/3*strcmp(awnPacket,"HV1")-...
           1/3*any(strcmp(awnPacket,["FHS","DM1","DM3","DM5","HV2","DV","EV4"]))); % Code rate
bitsPerSymbol = 1+ strcmp(awnSignalType,"EDR2M") + 2*(strcmp(awnSignalType,"EDR3M")); % Number of bits per symbol
snr = EbNo + 10*log10(codeRate) + 10*log10(bitsPerSymbol) - 10*log10(sps);


% Classify the channels for every |numPacketsToClassify| packets. If the PER of the
% channel is greater than |thresholdPER|, then map the corresponding channel
% as bad.
numPacketsToClassify = 50;
thresholdPER = 0.2; 

% Create an instance of the error rate
errorRate = comm.ErrorRate;

% Initialize variables to perform the simulation
numErrors = 0;
numPktLost = 0;
countPER = 0;
countPreviousPER = 0;
midFrequency = 2440e6;
if strcmp(awnFrequencyHopping,"On")
    errorsBasic = deal(zeros(numBTChannels,3));
    errorsBasic(:,1) = (0:numBTChannels-1);
end

% Number of interfering nodes that collide with AWN
collisionCount = 0;
for index = 1:iwnConfig.NumIWNNodes
    collisionCount = collisionCount + (iwn(index).CollisionProbability > 0);
end

if strcmp(awnFrequencyHopping,"On") && collisionCount ~= 0
    sinr = zeros(numPackets,1);
end

% Loop to simulate multiple packets
for inum = 1:numPackets

    % Generate AWN waveform
    if any(strcmp(awnSignalType,["LE1M","LE2M","LE500K","LE125K"]))
        if strcmp(awnFrequencyHopping,"On")
            channelIndex = frequencyHop();
            channelFrequencies = [2404:2:2424 2428:2:2478 2402 2426 2480]*1e6;
            awnFrequency = channelFrequencies(channelIndex+1);
        end
        txBits = randi([0 1],payloadLength*8,1,"int8");
        awnWaveform = bleWaveformGenerator(int8(txBits),Mode=awnSignalType,ChannelIndex=channelIndex, ...
            SamplesPerSymbol=sps,AccessAddress=accessAddBits,DFPacketType=awnPacket);
    else
        if strcmp(awnFrequencyHopping,"On")
            inputClock = inputClock + clockTicks;

            % Frequency hopping
            
            awnFrequency = (2402+channelIndex)*1e6;

            % Generate whiten initialization vector from clock
            clockBinary = int2bit(inputClock,28,false).';
            awnWaveformConfig.WhitenInitialization = [clockBinary(2:7)'; 1];
        end
        txBits = randi([0 1],payloadLength*8,1);
        awnWaveform = bluetoothWaveformGenerator(txBits,awnWaveformConfig);
    end
    
    % Add timing offset
    timingOffset = randsrc(1,1,1:0.1:100);
    timingOffsetWaveform = helperBLEDelaySignal(awnWaveform,timingOffset);

    % Add frequency offset
    freqOffsetImp = randsrc(1,1,-10e3:100:10e3);
    freqOffsetWaveform = helperBLEFrequencyOffset(timingOffsetWaveform,sampleRate,freqOffsetImp);
    
    % Add DC offset
    dcValue = (5/100)*max(freqOffsetWaveform);
    dcWaveform = freqOffsetWaveform + dcValue;

    % Shift the waveform by making 2440 MHz as the mid frequency
    freqOffset = awnFrequency-midFrequency;
    hopWaveform = helperBLEFrequencyOffset(dcWaveform,sampleRate,freqOffset);

    % Scale the waveform as per the transmitter power and path loss
    soiAmplitudeLinear = 10^((awnTxPower-30)/20)/awnPathloss;
    attenAWNWaveform = soiAmplitudeLinear*hopWaveform;

    freqOffsetPhase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    
    maxFreqShift = (sampleRate-awnBandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffsetPhase.FrequencyOffset = freqOff;
    attenAWNWaveform = freqOffsetPhase(attenAWNWaveform);
    
    iwnWaveformAlt = iwnWaveformPL;
    
    if rand_rge >= 1
    freqOffset1.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(1).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset1.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{1} = freqOffset1.Phase(iwnWaveformAlt{1});
end

if rand_rge >= 2
    freqOffset2.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(2).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset2.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{2} = freqOffset2.Phase(iwnWaveformAlt{2});
end

if rand_rge >= 3
    freqOffset3.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(3).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset3.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{3} = freqOffset3.Phase(iwnWaveformAlt{3});
end

if rand_rge >= 4
    freqOffset4.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(4).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset4.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{4} = freqOffset4.Phase(iwnWaveformAlt{4});
end

if rand_rge >= 5
    freqOffset5.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(5).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset5.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{5} = freqOffset5.Phase(iwnWaveformAlt{5});
end

if rand_rge >= 6
    freqOffset6.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(6).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset6.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{6} = freqOffset6.Phase(iwnWaveformAlt{6});
end

if rand_rge >= 7
    freqOffset7.Phase = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);
    maxFreqShift = (sampleRate-iwn(7).Bandwidth) / 2 - sampleRate/numFreqPixels;
    freqOff = (2*rand()-1)*maxFreqShift;
    freqOffset7.Phase.FrequencyOffset = freqOff;      
    iwnWaveformAlt{7} = freqOffset7.Phase(iwnWaveformAlt{7});
end
    

    % Add IWN waveforms to AWN waveform
    addIWN2AWN = addInterference(iwnConfig,attenAWNWaveform,iwnWaveformAlt,0);

    % Frequency shift the waveform by |-freqOffset|
    freqShiftWaveform = helperBLEFrequencyOffset(addIWN2AWN,sampleRate,-freqOffset);

    % Add AWGN
    soiPower = 20*log10(soiAmplitudeLinear);
    noisePower = soiPower - snr;
    splusibyn = 10*log10(var(freqShiftWaveform))-noisePower;
    noisyWaveform = awgn(freqShiftWaveform,splusibyn,"measured");

    % Apply filter
    if rem(length(noisyWaveform),sps)
        remainder = sps-rem(length(noisyWaveform),sps);
        noisyWaveform = [noisyWaveform;zeros(remainder,1)]; %#ok<AGROW> 
    end
    delay = floor(length(firdec.Numerator)/(2*decimationFactor));
    noisyWaveformPadded = [noisyWaveform;zeros(delay*decimationFactor,1)];
    filteredWaveform = firdec(noisyWaveformPadded);
    release(firdec)
    filteredWaveform = filteredWaveform(1+delay:end)*sqrt(decimationFactor);

    % Recover the data bits
    if any(strcmp(awnSignalType,["LE1M","LE2M","LE500K","LE125K"]))
        rxCfg.ChannelIndex = channelIndex;
        [rxBits,accAddress] = helperBLEPracticalReceiver(filteredWaveform,rxCfg);
        if isempty(rxBits) || ~isequal(accessAddBits,accAddress)
            pktStatus = [];
        end
    else
        % Get PHY properties
        rxCfg.WhitenInitialization = awnWaveformConfig.WhitenInitialization;
        [rxBits,~,pktStatus]...
                            = helperBluetoothPracticalReceiver(filteredWaveform,rxCfg);
    end
   % Compute BER and PER
    lengthTx = length(txBits);
    lengthRx = length(rxBits);
    lengthMinimum = min(lengthTx,lengthRx)-1;
    countPreviousPER = countPER;
    if lengthTx && lengthRx
        vectorBER = errorRate(txBits(1:lengthMinimum),rxBits(1:lengthMinimum));
        currentErrors = vectorBER(2)-numErrors;    % Number of errors in current packet
        if currentErrors || (lengthTx ~= lengthRx) % Check if current packet is in error or not
            countPER  = countPER+1;                % Increment the PER count
        end
        numErrors = vectorBER(2);
    elseif ~isempty(pktStatus)
        countPER  = countPER+~pktStatus;           % Increment the PER count
    else
        numPktLost = numPktLost+1;
    end

    % Perform frequency hopping
    if strcmp(awnFrequencyHopping,"On")
        chIdx = channelIndex+1;
        if countPreviousPER ~= countPER
            errorsBasic(chIdx,3) = errorsBasic(chIdx,3)+1;
        end

        % Classify the channels
        if any(inum == (1:floor(numPackets/numPacketsToClassify))*numPacketsToClassify)
            channelMap = errorsBasic(:,3)/numPacketsToClassify > thresholdPER;
            if nnz(channelMap) == 0
                continue;
            end
            badChannels = find(channelMap)-1;
            if length(frequencyHop.UsedChannels)-length(badChannels) < minChannels
                errorsBasic(badChannels+1,3) = 0;
                usedChannels = 0:36;
            else
                errorsBasic(badChannels+1,3) = 0;
                usedChannels = setdiff(frequencyHop.UsedChannels,badChannels);
            end
            frequencyHop.UsedChannels = usedChannels;
        end
    end
    
        % Visualize the spectrum and spectrogram. Compute SINR.
        if strcmp(awnFrequencyHopping,"On") && collisionCount ~= 0
            sinr(inum) = helperBluetoothSINREstimate(snr,awnTxPower,awnFrequency,pathlossdB,iwnConfig,iwnPathloss);
            % spectrumAnalyzer(iwnWaveform{1})
        elseif (strcmp(awnFrequencyHopping,"Off") && inum < 70) || (strcmp(awnFrequencyHopping,"On") && collisionCount == 0)
            if inum == 1
                sinr = helperBluetoothSINREstimate(snr,awnTxPower,awnFrequency,pathlossdB,iwnConfig,iwnPathloss);
            end
            
            
        end
end
freqOffset = comm.PhaseFrequencyOffset(...
  'SampleRate',sampleRate);

% Define WLAN channels in MHz
wlan_channels = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472];

% Choose random WLAN channel
channel_idx = randi(length(wlan_channels));

freqOff = wlan_channels(channel_idx) * 1e6 - 2.402e9;
freqOffset.FrequencyOffset = freqOff;
% Shift waveform using freqOffset function
%txWaveInt = freqOffset(rxBits);
%txWaveInt = circshift(rxBits,floor(timeShift*1e-3*sampleRate));

txWaveInt= freqShiftWaveform;
startFreq = awnFrequency*1e6 - sampleRate/2 + awnBandwidth/2;
waveInfo.Bandwidth =  [-awnBandwidth/2 +awnBandwidth/2;]';
FrequencyOffsets = [startFreq startFreq+awnBandwidth/2 + awnFrequency];
label = {'BT'};
for i = 1:rand_rge
    
    label{i+1} = 'BT';
end

waveInfo.freqPos = FrequencyOffsets + awnBandwidth;
waveInfo.SampleRate = sampleRate;



end