function Igray = rgb2gray2(Irgb)

[~, ~, numberOfColorChannels] = size(Irgb);
if numberOfColorChannels > 1
    Igray = rgb2gray(Irgb);
else
    Igray = Irgb;
end

end