%¶ÁÈ¡NOFFÎÄ¼þ
function [verts,normals]=read_noff(filename)
fid=fopen(filename,'r');
if(fid==-1)
    error('can''t open the file');
    return;
end
str=fgetl(fid);
a=fscanf(fid,'%d %d\n',[1,2]);
if a(1,2)==1
    for i=1:a(1,1)
        verts(i,:)=fscanf(fid,'%f %f %f',[1,3]);
        normals(i,:)=fscanf(fid,'%f %f %f',[1,3]);
        colors(i,:)=fscanf(fid,'%f %f %f',[1,3]);
    end
else
    for i=1:a(1,1)
        verts(i,:)=fscanf(fid,'%f %f %f',[1,3]);
        normals(i,:)=fscanf(fid,'%f %f %f',[1,3]);
    end
end
fclose(fid);