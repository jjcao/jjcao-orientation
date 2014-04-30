classdef GS
    % global settings    
    properties (Constant)
%         PC_COLOR = {1, .73, .0, 0.5f}; // gold
%         PC_COLOR = {0.275, .337, .60, 0.5f}; // blue
        PC_COLOR = [1.0, .65, .35]; % point cloud color, orange
        MIN_NEIGHBOR_NUM = 8;
        MAX_NEIGHBOR_NUM = 30;% orginal 30
        
        LAPLACIAN_CONSTRAINT_WEIGHT = 2; % init Laplacian contraction weight, compute automatically now.
        POSITION_CONSTRAINT_WEIGHT = 2; % init position constraint weight
    end
    
    methods (Static)
        function num = compute_k_knn(npts)
            num = max(GS.MIN_NEIGHBOR_NUM,round(npts*0.012));
            if num > GS.MAX_NEIGHBOR_NUM
                num = GS.MAX_NEIGHBOR_NUM;
            end
        end
        function [pts] = normalize(pts)
            % scale to unitBox and move to origin
            bbox = [min(pts(:,1)), min(pts(:,2)), min(pts(:,3)), max(pts(:,1)), max(pts(:,2)), max(pts(:,3))];
            c = (bbox(4:6)+bbox(1:3))*0.5;            
            pts = pts - repmat(c, size(pts,1), 1);
            s = 1.6 / max(bbox(4:6)-bbox(1:3));% make the bbox's diagnol = 1.6. %1.0, 1.6
            pts = pts*s;
        end
        function [] = test_normalize(pts)
            pts = GS.normalize(pts);
            [bbox, diameter] = GS.compute_bbox(pts);
            tmp = max(bbox(4:6)-bbox(1:3));
            if abs(tmp-1.0) > eps
                warning('GS.normalize() is wrong! the bounding box diagonal is: "%s" ', tmp);
            end
        end        
        function [bbox, diameter] = compute_bbox(pts)
            bbox = [min(pts(:,1)), min(pts(:,2)), min(pts(:,3)), max(pts(:,1)), max(pts(:,2)), max(pts(:,3))];
            rs = bbox(4:6)-bbox(1:3);
            diameter = sqrt(dot(rs,rs));
        end
        function wl = compute_init_laplacian_constraint_weight(PS,type)
            switch lower(type)
                case 'distance'
                    warning('not implemented!'); 
                case 'spring'
                    warning('not implemented!');                  
                case {'conformal','dcp'} % conformal laplacian  
                    ms = GS.one_ring_size(PS.pts,PS.rings,2);
                    wl = 1.0/(5.0*mean(ms));  
                otherwise % combinatorial or mvc
                    ms = GS.one_ring_size(PS.pts,PS.rings,2);
                    wl = 1.0/(5.0*mean(ms));
%                    wl = max(3, 0.1*(npts)^0.5); % original guess of JJCAO
%                    wl = 1.0/(10.0*sqrt(GS.average_face_area(PS.pts,PS.faces))); % implementation of Oscar08    
            end
        end
        function ms = one_ring_size(pts,rings,type)
            n = size(pts,1);
            ms = zeros(n,1);
            switch type
                case 1
                    parfor i = 1:n
                        ring = rings{i};    
                        tmp = repmat(pts(i,:), length(ring),1) - pts(ring,:);
                        ms(i) = min( sum(tmp.^2,2).^0.5);
                    end
                case 2
                    parfor i = 1:n
                        ring = rings{i};    
                        tmp = repmat(pts(i,:), length(ring),1) - pts(ring,:);
                        ms(i) = mean( sum(tmp.^2,2).^0.5);
                    end                    
                case 3
                    parfor i = 1:n
                        ring = rings{i};    
                        tmp = repmat(pts(i,:), length(ring),1) - pts(ring,:);
                        ms(i) = max( sum(tmp.^2,2).^0.5);
                    end                    
            end
        end
    end
end