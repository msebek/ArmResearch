function [bins] = histnc(data, edges)
% HISTNC Creates a N-dimensional histogram given edges.
%   [bins] = histnc(data, edges), takes N X D matrix data where each row is
%   a datapoint, and D-element cell array edges, where edges{i} is a Bi element
%   row containing the bin edges for dimension i. Returns a B1 X B2 X ... X
%   BD matrix containing counts.

N = size(data, 1);
D = size(data, 2);

% Generate the dimension-wise indices for bins
numBins = zeros(1, D);
binSubs = zeros(N, D);
for i = 1:D
    ed = edges{i};
    numBins(i) = numel(ed) - 1;
    [~, binSubs(:,i)] = histc(data(:,i), ed);
end

% Count the data now
bins = zeros(numBins);
binInds = num2cell(binSubs);
for i = 1:N
    bins(binInds{i,:}) = bins(binInds{i,:}) + 1;
end

end