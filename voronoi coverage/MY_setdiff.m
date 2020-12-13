function Z = MY_setdiff(X,Y)

if ~isempty(X)&&~isempty(Y)
  check = false(1, max(max(X), max(Y)));
  check(X) = true;
  check(Y) = false;
  Z = X(check(X));  
else
  Z = X;
end
