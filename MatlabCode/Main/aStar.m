% Run Astar algorithm
%
% adjMat - contains distances between vertices. If adjMat(3,4)=5.2, the
% distance between nodes 3 and 4 is 5.2. If adjMat(3,4) = 0, 3 and 4 are
% not connected
% heuristicVec - contains heuristic estimate for each node. If
% heuristicVec(5) = 4, then the heuristic estimate for node 5 is 4
% start - node to start from
% finish - node to finish from
function [pathdist, path, pred]=aStar(adjMat,h_list,start,target)
    % initialize lists
    % OPEN LIST STRUCTURE: 
    % IS ON 1/0 | node | parent node | h(n) | g(n) | f(n)
    open_list =[];
    OPEN_ISON=1;OPEN_NODE=2;OPEN_PARENT=3;OPEN_H=4;OPEN_G=5;OPEN_F=6;
    % CLOSED LIST STRUCTURE: 
    % node 
    closed_list =[];
    % NEIGHBOR LIST STRUCTURE: 
        % node | h(n) | g(n) | f(n)
     NEIGHBOR_NODE=1;NEIGHBOR_H=2;NEIGHBOR_G=3;NEIGHBOR_F=4;   
    
    %first open list is start
    open_count = 1;
    path_cost = 0;
    goal_dist = h_list(start);
    open_list(open_count,:) = insert_open(start,NaN,path_cost,goal_dist,path_cost+goal_dist);
    % set start as current node, 
    current = start
    % pop from open
    open_list(open_count,OPEN_ISON) = 0;
    % add to closed
    closed_count = 1;
    closed_list(closed_count) = current;
    
    no_path = 0;
    % DO MAIN LOOP
    while (current~=target && ~no_path)
        %expand around currentNode
        save('state.mat','current','closed_list','open_list')
        neighbor_list = get_neighbors(current,path_cost, closed_list,adjMat,h_list)
        neighbor_count = size(neighbor_list,1);
        % OPEN LIST STRUCTURE: 
        % IS ON 1/0 | node | parent node | h(n) | g(n) | f(n)
        % NEIGHBOR LIST STRUCTURE: 
        % node | h(n) | g(n) | f(n)
        for ii = 1:neighbor_count
            % check if neighbor is in open
            is_in_open = false;
            for jj = 1:open_count
                % if neighbor is already on open
                if neighbor_list(ii,NEIGHBOR_NODE) == open_list(jj,OPEN_NODE)
                    
                    % update cost if found shortcut
                    open_list(jj,OPEN_F)=min(open_list(jj,OPEN_F),neighbor_list(ii,NEIGHBOR_F));
                    %if updated
                    if open_list(jj,OPEN_F)==neighbor_list(ii,NEIGHBOR_F)
                                             
                        %update parent
                        open_list(jj,OPEN_PARENT)=current;
                        %update cost
                        open_list(jj,OPEN_H)=neighbor_list(ii,NEIGHBOR_H);
                        open_list(jj,OPEN_G)=neighbor_list(ii,NEIGHBOR_G);
                    end %end if updated
                    is_in_open = true;
                end % end check
            end % end open scan
            % if not found, add
            if ~is_in_open
                open_count = open_count+1;
%                 insert = insert_open(neighbor_list(ii,NEIGHBOR_NODE),...
%                     current,neighbor_list(ii,NEIGHBOR_H),neighbor_list(ii,NEIGHBOR_G),neighbor_list(ii,NEIGHBOR_F))
                open_list(open_count,:)=insert_open(neighbor_list(ii,NEIGHBOR_NODE),...
                    current,neighbor_list(ii,NEIGHBOR_H),neighbor_list(ii,NEIGHBOR_G),neighbor_list(ii,NEIGHBOR_F));
            end % end if not found
        end % end neighbor scan
        
        %find new min fn to be current node
        index_min_node = min_fn(open_list,open_count,target);
        % if new node found
        if (index_min_node ~= -1)
            %pop current
            current = open_list(index_min_node,OPEN_NODE)
            path_cost = open_list(index_min_node,OPEN_H);
            %move to closed
            closed_count = closed_count+1;
            closed(closed_count) = current;
            open_list(index_min_node,OPEN_ISON)=0;
        else %no path
            no_path = 1;
        end % end min node check
        open_list = open_list
    end % end while
    
    % Retrace path
    ii = 1;
    path(ii) = current;
    open_index = get_open_index(current,open_list);
    pathdist = open_list(open_index,OPEN_F);
    %Find current in open list
    while current ~= start && open_index~=-1 && ii<10
        open_index = get_open_index(current,open_list);
        current = open_list(open_index,OPEN_PARENT);
        ii = ii+1;
        path(ii) = current;
    end
    path = fliplr(path);
    pred = [];
    
    for ii=1:length(adjMat)
        open_index = get_open_index(ii,open_list);
        if open_index == -1
            pred(ii)=NaN;
        else
            pred(ii) = open_list(open_index,OPEN_PARENT);
        end
    end
    
end

function new_row = insert_open(node,parent_node,hn,gn,fn)
%Function to Populate the OPEN LIST
%OPEN LIST FORMAT
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |node|parent|h(n) |g(n)|f(n)|
%-------------------------------------------------------------------------
%
%  Modified Mathworks example
%   
new_row=[1,6];
new_row(1,1)=1;
new_row(1,2)=node;
new_row(1,3)=parent_node;
new_row(1,4)=hn;
new_row(1,5)=gn;
new_row(1,6)=fn;

end

function i_min = min_fn(open_list,open_count,target)
%Function to return the Node with minimum fn
% This function takes the list OPEN as its input and returns the index of the
% node that has the least cost
%
%   Modified mathworks example

 temp_array=[];
 kk=1;
 target_flag=false;
 goal_index=0;
 for jj=1:open_count
     if (open_list(jj,1)==1)
         temp_array(kk,:)=[open_list(jj,:) jj]; %#ok<*AGROW>
         if (open_list(jj,2)==target)
             target_flag=true;
             goal_index=jj;%Store the index of the goal node
         end;
         kk=kk+1;
     end;
 end;%Get all nodes that are on the list open
 if target_flag % one of the successors is the goal node so send this node
     i_min=goal_index;
 end
 %Send the index of the smallest node
 if size(temp_array ~= 0)
  [min_fn,temp_min]=min(temp_array(:,6));%Index of the smallest node in temp array
  i_min=temp_array(temp_min,7);%Index of the smallest node in the OPEN array
 else
     i_min=-1;%The temp_array is empty i.e No more paths are available.
 end
end
 
 
function index = get_open_index(current,open_list)
    index = 1;
    found = (open_list(index,2)==current);
    while index<size(open_list,1) && ~found
        index=index+1;
        found = (open_list(index,2)==current);
    end
    if index==size(open_list,1)
        index = -1;
    end 
end

function neighbors = get_neighbors(current,path_cost, closed_list,adjMat,h_list)
    neighbors = [];
    neighbor_count = 1;
    [~,col]=find(adjMat(current,:)>0);
    potentials=col';
    for ii = 1:length(potentials)
        on_closed = false;
        for jj = 1:length(closed_list)
            if potentials(ii) == closed_list(jj)
                on_closed = true;
            end
        end
        if ~on_closed
            neighbors(neighbor_count,1)=potentials(ii);
            neighbors(neighbor_count,2)=path_cost+adjMat(current,potentials(ii));
            neighbors(neighbor_count,3)=h_list(potentials(ii));
            neighbors(neighbor_count,4)=neighbors(neighbor_count,2)+neighbors(neighbor_count,3);
            neighbor_count=neighbor_count+1;
        end     
    end 
end