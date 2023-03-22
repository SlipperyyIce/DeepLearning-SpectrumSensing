function I = helperSpecSenseSpectrogramImage(x,Nfft,sr,imgSize)
%helperSpecSenseSpectrogramImage Spectrogram image from baseband signal
%   I = helperSpecSenseSpectrogramImage(X,NFFT,FS,IMGSZ) calculates the
%   spectrogram of baseband signal X, using NFFT length FFT and assuming
%   sample rate of FS. 

%   Copyright 2021 The MathWorks, Inc.

% Generate spectrogram
window = hann(256);
overlap = 10;

% Determine frequency range for spectrogram
BW = 4e9; % Set bandwidth to 100 MHz
fMin = 0;
    fMax = 16e9;

fVec = linspace(0, 8e9,Nfft);
[~,~,~,P] = spectrogram(x,window,overlap,...
  Nfft,sr,"centered",'psd');

P = 10*log10(abs(P')+eps);

% Plot spectrogram
%t = linspace(0, length(x)/sr, size(P, 2));
%f = linspace(0, 8e9, size(P, 1));

%imagesc(t, f, P);
%axis xy; % Flip the y-axis to show low frequencies at the bottom
%ylabel('Frequency (Hz)');
%xlabel('Time (s)');
%colorbar;


% Rescale pixel values to the interval [0,1]. Resize the image to imgSize
% using nearest-neighbor interpolation.
im = imresize(im2uint8(rescale(P)),imgSize,"nearest");

% Conver to RGB image with parula colormap. 
I = im2uint8(flipud(ind2rgb(im,parula(256))));

end