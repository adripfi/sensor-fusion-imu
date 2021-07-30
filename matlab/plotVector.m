function plotVector(t, vec, plotNorm, labels, normLabel, linestyle)
    if nargin < 3
        plotNorm = true;
    end
    if nargin < 4
        labels = {'x', 'y', 'z'};
    end
    if nargin < 5
        normLabel = 'norm';
    end
    if nargin < 6
        linestyle = '';
    end
    if size(vec, 2) ~= length(labels)
        error(['label count (' num2str(length(labels)) ') does not match vector column count (' num2str(size(vec, 2)) ')']);
    end
    
    hold all;
    for i=1:size(vec, 2)
        plot(t, vec(:,i), linestyle, 'DisplayName', labels{i});
    end
    if plotNorm
        vecNorm = sqrt(sum(vec.^2, 2));
        lh=plot(t, vecNorm, ['k' linestyle], 'DisplayName', normLabel);
        lh.Color=[0,0,0,0.3];
    end
end

