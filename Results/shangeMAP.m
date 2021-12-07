map=[1 1 1 1 1 1 1 1 1 1 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 0 0 1 1 1 0 0 0 0 1;
     1 0 2 0 0 1 3 0 0 0 1;
     1 0 0 1 1 1 0 0 0 0 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 0 0 0 0 0 0 0 0 0 1;
     1 1 1 1 1 1 1 1 1 1 1];
     DrawMap(map);         %得到环境地图

     %读取文件全部内容
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%建立环境地图%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function DrawMap(map)
n = size(map);
step = 1;
a = 0 : step :n(1);
b = 0 : step :n(2);
A = importdata('BSA_rounte_easyMap.csv');
Y = -A(:,1)+10.5;
X = A(:,2)+0.5;
plot(X,Y,'LineWidth',1,'Color','b')
figure(1)
axis([0 n(2) 0 n(1)]); %设置地图横纵尺寸
set(gca,'xtick',a,'ytick',b,'GridLineStyle','-',...
'xGrid','on','yGrid','on');
hold on
r = 1;
for(i=1:n(1))         %设置障碍物的左下角点的x,y坐标
    for(j=1:n(2))
        if(map(i,j)==1)
            % text(i-0.5,j-0.5,'1','Color','w')
            p(r,1)=j-1;
            p(r,2)=i-1;
            fill([p(r,1) p(r,1) + step p(r,1) + step p(r,1)],...
                 [p(r,2) p(r,2) p(r,2) + step p(r,2) + step ],'k');
            r=r+1;
            hold on
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%栅格数字标识%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold on
axis square
end
