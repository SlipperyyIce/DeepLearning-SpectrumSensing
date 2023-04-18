classdef helperIWNConfig < ble.internal.ConfigBase
%helperIWNConfig Interference wireless node (IWN) configuration parameters
%
%   IWNCONFIG = helperIWNConfig creates a default IWN configuration object,
%   IWNCONFIG.
%
%   IWNCONFIG = helperIWNConfig(Name=Value) creates an IWN configuration
%   object, IWNCONFIG, with the specified property Name set to the
%   specified Value. You can specify additional name-value arguments in any
%   order as (Name1=Value1,...,NameN=ValueN).
%
%   helperIWNConfig methods:
%   generateIWNWaveform - Generate IWN waveform(s)
%   computeBandwidth    - Compute IWN bandwidth(s)
%   applyPathloss       - Apply path loss to the IWN waveform(s)
%   addInterference     - Add interference to the AWN waveform
%
%   helperIWNConfig properties:
%
%   IWN                 - IWN properties of multiple nodes
%   SampleRate          - Sample rate
%   Environment         - Environment in which the signal propagates

%   Copyright 2021 The MathWorks, Inc.

properties
    %IWN IWN properties of multiple nodes
    %   Specify the IWN as 1-by-N structure with these five fields
    %   {SignalType, TxPosition, Frequency, TxPower, CollisionProbability}.
    %   N represents the number of interference nodes.
    IWN

    %SampleRate Sample rate of the signal in Hz
    %   Specify the sample rate as a positive double value in Hz. The
    %   default value is 88e6.
    SampleRate = 88e6

    %Environment Environment in which the signal propagates
    %   Specify the environment as one of 'Outdoor'|'Industrial'|'Home'|
    %   'Office'. The default value is 'Outdoor'.
    Environment = 'Outdoor'
end

properties (Hidden)
    %NumIWNNodes Number of IWN nodes
    NumIWNNodes

    %Bandwidth Bandwidth of the signal in Hz
    Bandwidth
end

methods
    % Constructor
    function obj = helperIWNConfig(varargin)
        obj = obj@ble.internal.ConfigBase(varargin{:});
        obj.NumIWNNodes = length(obj.IWN); % Number of IWN nodes
        obj.Bandwidth = computeBandwidth(obj); % Compute IWN signal bandwidth
    end

    function iwnWaveform = generateIWNWaveform(obj)
    %generateIWNWaveform Generate IWN waveform(s)
    %   IWNWAVEFORM = generateIWNWaveform(OBJ) returns the generated IWN
    %   waveform(s).
    %
    %   IWNWAVEFORM is a cell array of size 1-by-N, where N represents the
    %   number of interference nodes. Each cell represents a column vector
    %   of size Ns-by-1, where Ns represents the number of time-domain
    %   samples.
    %
    %   OBJ is an object of type helperIWNConfig.

        iwnWaveform = cell(1,obj.NumIWNNodes);
        % Generate multiple IWN waveforms
        for  i = 1:obj.NumIWNNodes
            switch obj.IWN(i).SignalType
                case {'LE1M','LE2M','LE500K','LE125K'}
                    % Derive channel index based on the center frequency
                    channelIndexArray = [37 0:10 38 11:36 39];
                    channelIndex = channelIndexArray((obj.IWN(i).Frequency-2402e6)/2e6+1);
                    payloadLength = 255;
                    data = randi([0 1],payloadLength*8,1,'int8'); 
                    phyFactor = 1+strcmp(obj.IWN(i).SignalType,'LE2M');
                    sps = obj.SampleRate/(phyFactor*1e6);
                    accAddress = '01234567';
                    accessAddBits = int2bit(hex2dec(accAddress),32,false);
                    waveform = bleWaveformGenerator(data,Mode=obj.IWN(i).SignalType,ChannelIndex=channelIndex,...
                                  SamplesPerSymbol=sps,AccessAddress=accessAddBits);
                case {'BR','EDR2M','EDR3M'}
                    bluetoothPacket = 'FHS';
                    sps = obj.SampleRate/1e6;
                    iwnWaveformConfig = bluetoothWaveformConfig(Mode=obj.IWN(i).SignalType,...
                                  PacketType=bluetoothPacket,SamplesPerSymbol=sps);
                    if strcmp(bluetoothPacket,'DM1')
                        iwnWaveformConfig.PayloadLength = 17;       % Maximum length of DM1 packets in bytes
                    end
                    dataLen = getPayloadLength(iwnWaveformConfig);  % Length of the payload
                    data = randi([0 1],dataLen*8,1);
                    waveform = bluetoothWaveformGenerator(data,iwnWaveformConfig);
                otherwise % WLAN waveform
                    if strcmp(obj.IWN(i).SignalType,'WLANHESUBandwidth20.bb')
                        bbReader = comm.BasebandFileReader(Filename='WLANHESUBandwidth20.bb');
                        bbInfo = info(bbReader);

                        % Configure the baseband file reader
                        bbReader.SamplesPerFrame = bbInfo.NumSamplesInData;

                        % Read the WLAN waveform from the baseband file
                        wlanWaveform = bbReader();
                        wlanSR = bbReader.SampleRate;
                    else
                        switch obj.IWN(i).SignalType
                            case '802.11b/g with 22 MHz Bandwidth'
                                psduLength = 2304;
                                cfgWLAN = wlanNonHTConfig(Modulation='DSSS',...
                                    DataRate='11Mbps',PSDULength=psduLength);
                            case '802.11g with 20 MHz Bandwidth'
                                psduLength = 2304;
                                cfgWLAN = wlanNonHTConfig(Modulation='OFDM', ...
                                    ChannelBandwidth='CBW20',PSDULength=psduLength);
                            case '802.11n with 20 MHz Bandwidth'
                                psduLength = 2304;
                                cfgWLAN = wlanHTConfig(ChannelBandwidth='CBW20',PSDULength=psduLength);
                            case '802.11n with 40 MHz Bandwidth'
                                psduLength = 2304;
                                cfgWLAN = wlanHTConfig(ChannelBandwidth='CBW40',PSDULength=psduLength);
                            case '802.11ax with 20 MHz Bandwidth'
                                cfgWLAN = wlanHESUConfig(ChannelBandwidth='CBW20');
                                psduLength = getPSDULength(cfgWLAN);
                            case '802.11ax with 40 MHz Bandwidth'
                                cfgWLAN = wlanHESUConfig(ChannelBandwidth='CBW40');
                                psduLength = getPSDULength(cfgWLAN);
                        end
                        data = randi([0 1], psduLength*8, 1); % Create a random PSDU
                        wlanWaveform = wlanWaveformGenerator(data,cfgWLAN);
                        wlanSR = wlanSampleRate(cfgWLAN);
                    end

                    % Filter the wlan waveform
                    TWInterpolation = 0.01;
                    AstopInterpolation = 40;
                    firinterp = dsp.FIRRateConverter(obj.SampleRate/1e6,wlanSR/1e6,...
                        designMultirateFIR(obj.SampleRate/1e6,wlanSR/1e6,...
                                           TWInterpolation,AstopInterpolation));
                    wlanWaveformRem = rem(length(wlanWaveform), wlanSR/1e6);
                    if wlanWaveformRem
                        wlanWaveformTemp = [wlanWaveform;zeros(wlanSR/1e6-wlanWaveformRem,1)];
                    else
                        wlanWaveformTemp = wlanWaveform;
                    end
                    waveform = firinterp(wlanWaveformTemp);
            end

            % Shift the waveform by making 2440 MHz as center frequency
            midFrequency = 2440e6;
            frequencyOffset = obj.IWN(i).Frequency - midFrequency;
            shiftedWaveform = helperBLEFrequencyOffset(waveform,obj.SampleRate,frequencyOffset);

            % Apply transmit power
            dBmConverter = 30;
            iwnWaveform{i}  = 10^((obj.IWN(i).TxPower-dBmConverter)/20)*shiftedWaveform;
        end
    end

    function bandwidth = computeBandwidth(obj)
    %computeBandwidth Compute bandwidth
    %   BANDWIDTH = computeBandwidth(OBJ) returns the bandwidth(s) of the
    %   interfering nodes.
    %
    %   BANDWIDTH is a vector of size 1-by-N, where N represents the number
    %   of interference nodes.
    %
    %   OBJ is an object of type helperIWNConfig.

        bandwidth = zeros(1,obj.NumIWNNodes);
        for  i = 1:obj.NumIWNNodes
            switch obj.IWN(i).SignalType
                case {'LE1M','LE2M','LE500K','LE125K'}
                    bandwidth(i) = 2e6;
                case {'BR','EDR2M','EDR3M'}
                    bandwidth(i) = 1e6;
                otherwise
                    switch obj.IWN(i).SignalType
                        case {'802.11b/g with 22 MHz Bandwidth','WLANBasebandFile'}
                            bandwidth(i) = 22e6;
                        case {'802.11g with 20 MHz Bandwidth',...
                              '802.11n with 20 MHz Bandwidth','802.11ax with 20 MHz Bandwidth'}
                            bandwidth(i) = 20e6;
                        case {'802.11n with 40 MHz Bandwidth','802.11ax with 40 MHz Bandwidth'}
                            bandwidth(i) = 40e6;
                    end
            end
        end
    end

    function [attenuatedWaveform,pldB] = applyPathloss(obj,iwnWaveform,awnRxPosition)
    %applyPathloss Apply path loss to the IWN waveform(s)
    %   [ATTENUATEDWAVEFORM,PLDB] = applyPathloss(OBJ,IWNWAVEFORM,...
    %   AWNRXPOSITION) returns the attenuated waveform by scaling the IWN
    %   waveform with the computed path loss.
    %
    %   ATTENUATEDWAVEFORM is a cell array of size 1-by-N, where N
    %   represents the number of interference nodes. Each cell represents a
    %   column vector of size Ns-by-1, where Ns represents the number of
    %   time-domain samples.
    %
    %   PLDB specifies the path loss in dB. It is a row vector of size
    %   1-by-N, where N represents the number of interference nodes.
    %
    %   OBJ is an object of type helperIWNConfig.
    %
    %   IWNWAVEFORM is a cell array of size 1-by-N, where N represents the
    %   number of interference nodes. Each cell represents a column vector
    %   of size Ns-by-1, where Ns represents the number of time-domain
    %   samples.
    %
    %   AWNRXPOSITION is a scalar which specifies the AWN receiver
    %   position.

        attenuatedWaveform = cell(1,obj.NumIWNNodes);
        pldB = zeros(1,obj.NumIWNNodes);
        for i = 1:obj.NumIWNNodes
            distAWNRxIWNTx = sqrt(sum((obj.IWN(i).TxPosition-awnRxPosition).^2,2));
            [pathloss,pathlossdB]= helperBluetoothEstimatePathLoss(obj.Environment,distAWNRxIWNTx);
            attenuatedWaveform{i} = iwnWaveform{i}/pathloss;
            pldB(i) = pathlossdB;
        end
    end

    function awnIWNWaveform = addInterference(obj,awnWaveform,iwnWaveform,timingDelay)
    %addInterference Add interference to the AWN waveform
    %   AWNIWNWAVEFORM = addInterference(OBJ,AWNWAVEFORM,IWNWAVEFORM,...
    %   TIMINGDELAY) adds IWN waveforms to the AWN waveform.
    %
    %   AWNIWNWAVEFORM is a complex column vector, representing the
    %   AWN waveform along with interference.
    %
    %   OBJ is an object of type helperIWNConfig.
    %
    %   AWNWAVEFORM is a complex column vector, representing the AWN
    %   waveform.
    %
    %   IWNWAVEFORM is a cell array of size 1-by-N, where N represents the
    %   number of interference nodes. Each cell represents a column vector
    %   of size Ns-by-1, where Ns represents the number of time-domain
    %   samples.
    %
    %   TIMINGDELAY is a scalar, representing the delay in AWN waveform.

        interWfm = zeros(obj.NumIWNNodes,length(awnWaveform));
        
        for j = 1:obj.NumIWNNodes
            
            if obj.IWN(j).CollisionProbability > 0 && obj.IWN(j).CollisionProbability <= 1
                numZerosAppended = ceil(timingDelay)+...
                    ceil(length(awnWaveform(ceil(timingDelay)+1:end))*(1-obj.IWN(j).CollisionProbability));
                iwnWaveformAppended = [zeros(numZerosAppended,1);iwnWaveform{j}];
                rng('shuffle');
                rng_delay = -rand() * 30100;
                iwnWaveformAppended = delayseq(iwnWaveformAppended,rng_delay);
                
                if length(iwnWaveformAppended) < length(awnWaveform)
                    iwnWaveformAppended = [iwnWaveformAppended;...
                        zeros(length(awnWaveform)-length(iwnWaveformAppended),1)];
                end
                interWfm(j,:) = iwnWaveformAppended(1:length(awnWaveform));
            end
        end
        awnIWNWaveform = sum(interWfm,1).'+awnWaveform;
    end
end
end