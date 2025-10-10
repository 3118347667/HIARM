-- RC filter
-- global
is_first = true
--
cutoff_freq = 40;
if (is_first) then
  is_first = false
  prevX = time
  prevY = value
end
dt = time-prevX;
prevX = time;
rc = 1.0 / (2 * 3.1415926 * cutoff_freq);
prevY=prevY+(dt / (dt + rc))*(value-prevY);

return prevY


-- Time delay
-- global
a={}
is_first=true
--
freq = 333
delay = 0.08
size = math.floor(freq*delay)+1
if (is_first) then
  is_first = false
  for i=1, size do
     a[i] = 0.0
  end
else
  for i=1, size-1 do
     a[i] = a[i+1]
  end
  a[size]=value
end
val = v1-a[1]
if (math.abs(val) > 1000) then
  val = 0
end
return val