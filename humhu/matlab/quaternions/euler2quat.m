function [q] = euler2quat(eul)

mats = euler2mat(eul);
q = mat2quat(mats);