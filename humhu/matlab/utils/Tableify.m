function [s] = Tableify( input )
% Converts the input cell or double matrix into a Latex table string

if isa( input, 'double' )
   input = num2cell( input );
   for i = 1:numel( input )
      input{i} = num2str( input{i} ); 
   end
end

w = size( input, 2 );
h = size( input, 1 );

s = [];
for i = 1:h
    for j = 1:w-1
        s = [s, input{i,j}, ' & '];
    end
    s = [s, input{i,w}, ' //\n'];
end

fprintf( s );