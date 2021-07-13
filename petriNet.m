classdef petriNet < handle
    %PETRINET A Petri net representing a discrete event system
    %   A Petri net class representing the dynamics of a given discrete
    %   event system. 
    
    properties (Access=public)
        x           % marking
        numPlaces   % number of places
        numTrans    % number of transitions
    end
    properties (Access=private)
        I           % input to transition
        O           % transition to output
        A           % Update Matrix
        x0          % initial construct marking
    end
    
    methods (Access = public)
        function obj = petriNet(I,O,x0)
            %PETRINET Construct an instance of petriNet class
            %   Given a input-transition relationship I , transition-output
            % relationship, and the initial marking, this constructor will
            % construct a Petri net that match the given specification.
            
            obj.x = x0;
            obj.x0 = x0;
            obj.I = I;
            obj.O = O;
            obj.A = O-I';
            [n,m] = size(I);
            obj.numPlaces = n;
            obj.numTrans = m;
        end
        
        function x_new = fire(obj,in)
            %FIRE fire the enabled Petri net's transitions
            %   Each transition will be fired if the firing condition is 
            % met based on the current marking of the Petri net and the
            % given control input IN.
            t_en = obj.checkTransEn;
            u = in.*t_en;
            obj.x = obj.x + u*obj.A;
            x_new = obj.x;
        end
        
        function reset(obj)
            %RESET revert the marking back to the original marking
            %   Revert the marking of the Petri net back to the initial
            %   marking (when the Petri net was instantiated)
            obj.x = obj.x0;
        end
        
        function t_en = checkTransEn(obj)
            %CHECKTRANSEN chech whether each transition is enabled
            m = obj.numTrans;
            t_en = zeros(1,m);
            x_current = obj.x;
            I_current = obj.I;
            for j = 1:m
                t_en(j) = all(x_current(I_current(:,j)>0)>=I_current(I_current(:,j)>0,j)');
            end
        end
        function [P_invariant,A] = Frakas(obj)
%             size(obj.A');
%             size(eye(obj.numPlaces));
%             clc;
            D = [obj.A' eye(obj.numPlaces)];
%             C = [-1 1 1 -1;1 -1 -1 1;0 0 1 0;1 0 0 -1;-1 0 0 1];
%             D = [C eye(5)];
            for i = 1:obj.numTrans%
                d_sub = D(1:end,i);
                list_del = [];
                for j = 1:numel(d_sub)
                    for k = j : numel(d_sub)
                        if(sign(d_sub(j))*sign(d_sub(k))== -1)
                            if ~(any(list_del(:)==j))
                                list_del(end+1) = j;
                            end
                            if ~(any(list_del(:)==k))
                                list_del(end+1) = k;
                            end
                            
                            d = abs(d_sub(k))*D(j,1:end) + abs(d_sub(j))*D(k,1:end);
                            di = d/gcd(sym(d));
                            D = [D ; di];
                        end
                    end 
                end
%                 i
                if(numel(list_del) == 0)
                    input = [1:numel(d_sub) find(d_sub'~=0)];
                    
                else
                    input = [1:size(D,1) list_del];
                end
%                 D;
%                 list_del;
                un_in = unique(input);
                n = hist(input,numel(un_in));
                oputput = un_in(n==1);
                D=D(oputput,1:end);
            end
            D = D (1:end,obj.numTrans+1:end);
%             size(D)
            D0 = unique(D,'rows');
%             size(D0)
            P_invariant = D0;
            A = obj.A;
%             obj.A*D0';
            
%             D1 = D0(find(D0(1:end,3)==1),1:end)
%             size(D1)
%             
%             D2 = D1(find(D1(1:end,8)==1),1:end)
%             size(D2)
            
          
        end
    end
  
end

