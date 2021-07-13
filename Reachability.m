%reachability graph
clear all;clc;tic
global t;
global name_t;
t = {};
name_t =[];
% P2T = {1,2,3,4,[1,7],[2,6],[3,5],8,7,6,5};
% T2P = {2,[3,5],[4,6],[1,7],10,[7,9],[6,8],[5,11]};
% x0 = [1,0 0 0,1 1 1,0 0 0,1];

% [P2T,T2P,x0] = Test1;
% save('a.mat','P2T','T2P');
% P2T = {1,2,5,7,[4 6 9],[3 8],11,12,10,3,[1 4],2,6,7,[5 10],8,[9 11],12};
% T2P = {[2 10],[3 11],[1 13],[2 14],[4 12],[6 14],[5 15],[7 13],[8 14],[4 18],[8 16],[9 17]};
% x0 = [1 0 0, 0 0 0, 0 0 0,0 1 1,1 1 1,1 1 1];
% 
% % All type without any policy
% P2T = {1,2,[],3,4,[]};
% T2P = {2,3,5,6};
% x0 = [1 0 0, 1 0 0];
% 
% % Type-1
% P2T = {[1,7,8],[2,5,6],[11,12,13],[9],[10],[3,5,6],[4,7,8],[9,10,13],[11],[12],[4],[1,3,9,10,11,12],[2],[5,7],[6,8],[9,10,11,12]};
% T2P = {[2 11],[3 12],[7 13],[8 12],[4,6,12,16],[5,6,12,16],[1,9,12,16],[1,10,12,16],[2,8,14],[2,8,15],[3,7,14],[3,7,15],[1,6]};
% x0 = [1 0 0 0 0, 1 0 0 0 0, 0 1 0, 1 1, 0];
% 
% % Type-1 with some policy
% P2T = {1,2,[5],3,4,[5],4,[1,3,5],2};
% T2P = {[2,7],[3,8],[5,9],[6,8],[1,4,8]};
% x0 = [1 0 0, 1 0 0,0 1 0];

% % Type-1 with some policy 2
% P2T = {1,2,[],3,4,[],4,[1,3],2};
% T2P = {[2,7],[3,8],[5,9],[6,8]};
% x0 = [1 0 0, 1 0 0,0 1 0];

% % Type-2
% P2T = {1,2,[5],3,4,[5],[5],[4],[1,3],[2],[5]};
% T2P = {[2,7],[3,9],[5,11],[6,9],[1,4,8,10]};
% x0 = [1 0 0, 1 0 0, 0 1 1 1 0];

% Type-3
P2T = {[1,5],[2],[6,7],[3],[4,5],[7],[6],[4],[1,3,6],[2],[7],[5],[6]};
T2P = {[2,8],[3,9],[5,11],[6,9],[1,7,9,13],[3,5,12],[1,4,10]};
x0 = [1 0 0, 1 0 0 0, 0 1 1 0, 1 0];

% % Type-3 with some policy
% P2T = {1,2,[5],3,4,[5],[],[4],[1,3],[2],[5]};
% T2P = {[2,8],[3,9],[5,11],[6,9],[1,4,10]};
% x0 = [1 0 0, 1 0 0, 0, 0 1 1 0];

% % InBook
% P2T = { 1,2,3,4,2,[3,1],1};
% T2P = { 2,[3,6],[4,5,7],[1,6]};
% x0 = [2 0 0 0 1 1 1];
% 
% P2T = { 1,2,3,4,2,[3,1]};
% T2P = { 2,[3,6],[4,5],[1,6]};
% x0 = [2 0 0 0 1 1];


n = numel(P2T);
m = numel(T2P);
I = zeros(n,m);
O = zeros(m,n);
for i = 1:n
    I(i,P2T{i}) = 1;
end
for j = 1:m
    O(j,T2P{j}) = 1;
end
 
tic
PetriNet = petriNet(I,O,x0);

u = zeros(1,PetriNet.numTrans);
update(PetriNet,u);
t_en = PetriNet.checkTransEn;
p ={PetriNet.fire(u)};
Dead = [];
G = digraph();
A =[];
B =[];
[Dead,p,A,B] = PrintTree(PetriNet,Dead,p,I,O,A,B);
G = addedge(G,A,B,name_t);
k = plot(G,'EdgeLabel',G.Edges.Weight,'EdgeLabelColor','#77AC30');
highlight(k,Dead,'NodeColor','r')
% h= plot(G,'MarkerSize',30,'LineWidth',1,'EdgeColor','b','NodeColor','r')
% % h.NodeColor ='';
% h.NodeFontSize = 20;
% h.LineWidth =5;
% h.ArrowSize =20;
% h.NodeLabelColor= 'k';
toc
function update(PetriNet,u)
          
    p_en = PetriNet.fire(u);
    t_en = PetriNet.checkTransEn;
    
end
function [Dead,p,A,B] = PrintTree(PetriNet,Dead,p,I,O,A,B)
    global t;
    global name_t;
    PetriNet_t = PetriNet;
    t_en = PetriNet_t.checkTransEn;
    A0 = zeros(1,numel(t_en));
    p_en1 = PetriNet_t.fire(zeros(1,PetriNet.numTrans));
    find(t_en);
    for i = find(t_en)
        i;
        A0(i) = 1;
        p_en = PetriNet_t.fire(A0);
        if any(cellfun(@isequal, p, repmat({p_en}, size(p))))
%             disp('Reeeeeeeeeeeeeee');
            t(end+1) = {A0};
             
        else
            t(end+1) = {A0};
%             name_t(end+1) = i; 
            p(end+1) = {p_en};
            if((PetriNet_t.checkTransEn)*ones(numel(PetriNet_t.checkTransEn),1)==0)
                Dead(end+1) = string(numel(p));
            end
            [Dead,p,A,B] = PrintTree(PetriNet_t,Dead,p,I,O,A,B);
        end
        name_t(end+1) = i;
        PetriNet_t = petriNet(I,O,p_en1);
        A0 = zeros(1,numel(t_en));
        A(end+1) = find(cellfun(@(p) isequal(p, p_en1), p));
        B(end+1) = find(cellfun(@(p) isequal(p, p_en), p));
    
    end
end
function [P2T,T2P,x0] = Test1
            A1 = [1 2 3 4 4 5 5 5 6 7 8 9];
            B1 = [2 3 6 1 7 2 4 8 5 8 9 6];
            if numel(A1) ~= numel(B1)
                return;
            end
            Trans = 1;
            P2T{max([A1 B1])*2} = [];
            T2P{numel(A1)*2} = [];
            for i = 1:numel(A1)
                 P2T{A1(i)}   = [ P2T{A1(i)} Trans];
                Trans= Trans+1;
                P2T{B1(i)}   = [P2T{B1(i)} Trans];
                Trans= Trans+1;
                T2P{2*i-1}  = [T2P{2*i-1} B1(i)];
                T2P{2*i}    = [T2P{2*i} A1(i)];
            end
            
%             Old_P2T = P2T;
            
            for j = (max([A1 B1])+1):(max([A1 B1])*2)
                for i = 1:numel(P2T{j-max([A1 B1])})
                    T2P{P2T{j-max([A1 B1])}(i)} = [T2P{P2T{j-max([A1 B1])}(i)} j];
                end
            end
            for j = 1:(max([A1 B1]))
                for i = 1:numel(T2P)
                    if any(find(T2P{i}==j))
                        P2T{(max([A1 B1])+j)}(end+1)  = i;
                    end
                end
            end
            x0 = [1 zeros(1,max([A1 B1])-1) 0 ones(1,max([A1 B1])-1)];
end