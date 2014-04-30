%可视化投票情况
function test_record_result(P)
%可视化投票结果
file_vote=fopen('../result/record_vote.txt','r');
num_vote=fscanf(file_vote,'%d\n',[1,1]);
vote_result=fscanf(file_vote,'%f\n');
fclose(file_vote);
co=find(vote_result==0);
x_vote=[1:num_vote];
%subplot(2,1,1),
%bar(x_vote,vote_result);%画出可视投票直方图
%title('the voting result of visible points');
%xlabel('the number of visible points');
%ylabel('voting weight');
%subplot(2,1,2),
[a b]=hist(vote_result,200);
%a=a/num_vote;
bar(b,a);
title('the distribution of visible points');
xlabel('the weight of visible points');
ylabel('the amount of visible points');
%max_index=find(b==max(b));
%min_index=find(b==min(b));
%g1=b(max_index);g2=b(min_index);
%max_g=max(b)-g1;
%min_g=min(b)+g2;
%max_b=sprintf('%f',max(b));
%min_b=sprintf('%f',min(b));
%max_g=sprintf('%f',max_g);
%min_g=sprintf('%f',min_g);
%str_max=['the confident interval is ',' ( ',max_g,' , ',max_b,' ) '];
%str_min=['the confident interval is ','( ',min_b,' , ',min_g,' ) '];
%text(b(max_index),a(max_index),str_max);
%text(b(min_index),a(min_index),str_min)