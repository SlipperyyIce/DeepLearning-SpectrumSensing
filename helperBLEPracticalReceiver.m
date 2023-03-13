function [bits,accessAddress] = helperBLEPracticalReceiver(rxWaveform,rxCfg)
%helperBLEPracticalReceiver Demodulate and decodes the received Bluetooth
%LE waveform
%
%   [BITS,ACCESSADDRESS] = helperBLEPracticalReceiver(RXWAVEFORM,RXCFG)
%   decodes the received Bluetooth LE waveform, RXWAVEFORM.
%
%   BITS is an int8 column vector containing the recovered information
%   bits with maximum length of 2080 bits.
%
%   ACCESSADDRESS is an int8 column vector of length 32 bits containing the
%   access address information.
%
%   RXWAVEFORM is a complex valued time-domain waveform with size Ns-by-1,
%   where Ns represents the number of received samples.
%
%   RXCFG is a structure containing these fields:
%   'Mode'                  Specify the physical layer reception mode as
%                           one of 'LE1M', 'LE2M', 'LE500K', and 'LE125K'.
%
%   'ChannelIndex'          Specify the channel index as an integer in
%                           the range [0,39]. For data channels,
%                           specify this value in the range [0,36]. For
%                           advertising channels, specify this value in
%                           the range [37,39].
%
%   'SamplesPerSymbol'      Specify the samples per symbol as a positive
%                           integer.
%
%   'DFPacketType'          Specify the direction finding packet type as
%                           'ConnectionlessCTE', 'ConnectionCTE', or
%                           'Disabled'.
%
%   'CoarseFreqCompensator' Specify the coarse frequency compensator system
%                           object as comm.CoarseFrequencyCompensator.
%
%   'PreambleDetector'      Specify the preamble detector system object as
%                           comm.PreambleDetector.

%   Copyright 2018-2021 The MathWorks, Inc.

% DC offset correction
rxDCFree = rxWaveform - mean(rxWaveform);

% Estimate and compensate frequency offset
rxFreqComp = rxCfg.CoarseFreqCompensator(rxDCFree);
release(rxCfg.CoarseFreqCompensator)

% Generate reference signals used for packet detection
sps = rxCfg.SamplesPerSymbol;
preamble = ble.internal.preambleGenerator(rxCfg.Mode,rxCfg.AccessAddress);  
if any(strcmp(rxCfg.Mode,{'LE1M','LE2M'}))
   refSequence = [preamble; rxCfg.AccessAddress];
else
    trellis = poly2trellis(4,[17 13]);
    fecAA = convenc(rxCfg.AccessAddress,trellis);
    pattern = [1 1 0 0].';
    patternLen = length(pattern);
    repBlock = reshape(repmat(fecAA.',patternLen,1),1,[]);
    repPattern = reshape(repmat(pattern,1,length(fecAA)),1,[]);
    codedAA = ~xor(repBlock,repPattern).';
    refSequence = [preamble; codedAA];
end
refSamples = ble.internal.gmskmod(refSequence,sps);

% Perform timing synchronization
prbDet = rxCfg.PreambleDetector;
prbDet.Preamble = refSamples;
[~, dtMt] = prbDet(rxFreqComp);
release(prbDet)
prbDet.Threshold = max(dtMt);
prbIdx = prbDet(rxFreqComp);
release(prbDet)
if prbIdx >= length(prbDet.Preamble)
    rcvTrim = rxFreqComp(1+prbIdx-length(prbDet.Preamble):end);
else
    rcvTrim = rxFreqComp;
end
if rem(length(rcvTrim),sps)
    rcvTrim = [rcvTrim;zeros(sps-rem(length(rcvTrim),sps),1)];
end

% Recover the data bits
fecBlock1Length = 40*any(strcmp(rxCfg.Mode,{'LE500K','LE125K'}));
cteInfoInd = 24*strcmp(rxCfg.DFPacketType,'ConnectionCTE')+40*strcmp(rxCfg.DFPacketType,'ConnectionlessCTE');
minPacketLength = length(refSamples)+(fecBlock1Length+cteInfoInd)*sps;
if length(rcvTrim) > minPacketLength
    if strcmp(rxCfg.Mode,'LE500K') && (rem(length(rcvTrim),2*sps) ~= 0)
        padLen = 2*sps - rem(length(rcvTrim),2*sps);
    elseif strcmp(rxCfg.Mode,'LE125K') && (rem(length(rcvTrim),8*sps) ~= 0)
        padLen = 8*sps - rem(length(rcvTrim),8*sps);
    else
        padLen = 0;
    end
    rcvTrim = [rcvTrim; zeros(padLen,1)];
    [bits,accessAddress] = bleIdealReceiver(rcvTrim,Mode=rxCfg.Mode,...
            ChannelIndex=rxCfg.ChannelIndex,SamplesPerSymbol=sps,...
            AccessAddress=rxCfg.AccessAddress,DFPacketType=rxCfg.DFPacketType);
else
    bits = [];
    accessAddress = [];
end
end