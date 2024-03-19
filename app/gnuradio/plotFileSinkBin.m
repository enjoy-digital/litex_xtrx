clear all;
close all;
fd = fopen("/tmp/toto.bin"); a = fread(fd, Inf, "float"); fclose(fd);

a1 = a(1:2:end) + i * a(2:2:end);
a2 = a1(1:2:end);
a3 = a1(2:2:end);
STEP = 4;
aa1 = a(1:STEP:end);
aa2 = a(2:STEP:end);
aa3 = a(3:STEP:end);
aa4 = a(4:STEP:end);
%plot(aa1, 'r', aa2, 'g', aa3, 'b', aa4, 'k');

STEP = 2;
aa1 = a(1:STEP:end);
aa2 = a(2:STEP:end);
plot(aa1, 'r', aa2, 'g');
%figure()
%plot(aa1, 'r');
%figure()
%plot(aa2, 'g');
