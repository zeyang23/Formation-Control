function uddott = interU(ut,udott)
uddott = zeros(1,length(udott));
for i=2:length(ut)
    uddott(i) = (udott(i)^2-udott(i-1)^2)/2/(ut(i)-ut(i-1));
    if isnan(uddott(i))
        uddott(i) = 0;
    end
end
end