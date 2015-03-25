function [appended] = AppendStructs( a, b )

names = fieldnames( a );

for i = 1:numel(names)
   
    aVal = a.(names{i});
    bVal = b.(names{i});
    
    if ~isa( aVal, 'double' )
        error( 'Cannot append non-doubles yet!' );
    end
    
    appended.(names{i}) = cat( numel(size(aVal)), aVal, bVal);
    
end