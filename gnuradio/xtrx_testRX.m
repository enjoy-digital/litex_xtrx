fd = fopen("toto.bin"); a = fread(fd, 1024, "float"); fclose(fd);

a1 = a(1:2:end) + i * a(2:2:end);
a2 = a1(1:2:end);
a3 = a1(2:2:end);
plot(imag(a2), 'r', real(a2)+5, 'g', imag(a3)+10, 'b', real(a3)+15, 'k')
%plot(imag(a1), 'r', real(a1)+10, 'g', imag(a2), 'b', real(a2), 'k')
