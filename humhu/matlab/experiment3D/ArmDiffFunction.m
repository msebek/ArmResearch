function [diff] = ArmDiffFunction( a, b )
c = a/b;
diff = c.GetCoordinates();