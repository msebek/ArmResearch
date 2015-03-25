function [R] = euler2mat(eul)

N = size(eul,2);

yawCos = cos(eul(1,:));
yawSin = sin(eul(1,:));
pitchCos = cos(eul(2,:));
pitchSin = sin(eul(2,:));
rollCos = cos(eul(3,:));
rollSin = sin(eul(3,:));

yawR = repmat( eye(3), [1,1,N] );
yawR(1,1,:) = yawCos;
yawR(2,2,:) = yawCos;
yawR(1,2,:) = -yawSin;
yawR(2,1,:) = yawSin;

pitchR = repmat( eye(3), [1,1,N] );
pitchR(1,1,:) = pitchCos;
pitchR(3,3,:) = pitchCos;
pitchR(1,3,:) = pitchSin;
pitchR(3,1,:) = -pitchSin;

rollR = repmat( eye(3), [1,1,N] );
rollR(2,2,:) = rollCos;
rollR(3,3,:) = rollCos;
rollR(2,3,:) = -rollSin;
rollR(3,2,:) = rollSin;

R = zeros(3,3,N);
for i = 1:N
%     R(:,:,i) = rollR(:,:,i)*pitchR(:,:,i)*yawR(:,:,i);
    R(:,:,i) = yawR(:,:,i)*pitchR(:,:,i)*rollR(:,:,i);
end