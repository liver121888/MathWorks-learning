function showsnap(snaps, id)
h = figure("Name","Snap #" + id,"IntegerHandle","off","Units","normalized","Position",[0.01 0.01 0.98 0.98]);
a = axes(h);
imshow(snaps{id}.cdata,"Parent",a)
end