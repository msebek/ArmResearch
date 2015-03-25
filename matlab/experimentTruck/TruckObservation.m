function [obs] = TruckObservation( x, landmark )

relPos = landmark - x(1:2,:);
obs = [ sqrt( sum( relPos(1:2,:).*relPos(1:2,:), 1 ) );
        wrapToPi(atan2( landmark(2,:) - x(2,:), landmark(1,:) - x(1,:) ) - x(3,:)) ];
    