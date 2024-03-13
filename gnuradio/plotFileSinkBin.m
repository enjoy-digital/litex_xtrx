fd = fopen("toto.bin"); a = fread(fd, 1024*4, "float"); fclose(fd);

a1 = a(1:2:end) + i * a(2:2:end);
a2 = a1(1:2:end);
a3 = a1(2:2:end);
plot(imag(a1), 'r', real(a1)+10)
