function monoplot(t, x, t_d, P_d, export)
    load('monopod_parameters', 'l_B', 'l_UL', 'l_LL', 'w_B', 'w_UL', 'w_LL')
    
    % frame update time scale.
    t_scale = 1;
    t_step = t_scale * t(end)/length(t)
    
    fig = figure;
    
    ylim([-1 8])
    line([-1000 1000], [0 0], 'color', 'b')
    line([-1000 1000], [1.5 1.5], 'color', 'b')
    
    color = [0.8 0.8 0.8];
    p_B = patch(zeros(4, 1), zeros(4, 1), color);
    p_UL = patch(zeros(4, 1), zeros(4, 1), color);
    p_LL = patch(zeros(4, 1), zeros(4, 1), color);
    p_F = patch(zeros(4, 1), zeros(4, 1),  [0.95 0.0 0.0]);
    p_com = patch(zeros(4, 1), zeros(4, 1),  [0.95 0.0 0.0]);
    
    % desire position indicator
    counter = 1;
    pos_des = P_d(counter);
    l = line([pos_des pos_des], [0 100], 'color', 'g');
    
    if export
        file_name = 'result.gif';
        if exist('img', 'dir')
            path_dest = append(pwd, filesep, 'img', filesep, file_name);
        else
            path_dest = append(pwd, filesep);
        end
    end
    %hold on
    %plot(x(:,1),x(:,2));
    for i = 1:length(t)
        hold on;
        % draw trajectory
        plot(x(1:i,1), x(1:i,2),'color', 'r');

        hold on;
        % store current tick time
        tic

        % get joint configuration
        q = x(i, 1:4)';
        
        % draw body parts
        p_B = transform_patch(p_B, l_B, w_B, get_W_X_B(q));
        p_UL = transform_patch(p_UL, l_UL, w_UL, get_W_X_UL(q));
        p_LL = transform_patch(p_LL, l_LL, w_LL, get_W_X_LL(q));
        p_F = transform_patch(p_F, 0.08, 0.08, get_W_X_F(q));
        
        % calculate world space com transformation matrix
        w_com = get_world_com(q);
        mat_com = [1, 0, 0, w_com(1);
        0,  1, 0, w_com(2);
         0, 0, 1, 0;
         0, 0, 0, 1];
        % draw com position
        p_com = transform_patch(p_com, 0.05, 0.05, mat_com);
        
        % change x axis range to display the body at the center of the figure.
        xlim([q(1)-7, q(1)+ 7]);
        
        if t(i) >= t_d(counter)
            counter = min(counter + 1, length(t_d));
            pos_des = P_d(counter);
            delete(l)
            l = line([pos_des pos_des], [0 100], 'color', 'g');
        end
        
        title(sprintf("t = %.2fs", t(i)))
        drawnow;
        
        if export
            frame = getframe(fig);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if i == 1
                imwrite(imind,cm, path_dest,'gif', 'Loopcount',inf, 'DelayTime', t_step); 
            else
                imwrite(imind,cm,path_dest,'gif','WriteMode','append', 'DelayTime',t_step);
            end
        end
        
        % get elapsed time from store tic point.
        t_elapsed = toc;
        pause(max(t_step - t_elapsed, 0))  % adjust pause such that plot updates in (near) real-time
    end
end


function p_trans = transform_patch(p, length, width, transform)
    % initialize vertices around world origin and transform to specified 
    % pose using W_X_*, update existing patch with transformed vertices
    vertices_homo = [-length/2, -length/2, length/2, length/2;
                      -width/2,   width/2,  width/2, -width/2;
                             0,         0,        0,        0;
                             1,         1,        1,        1];
    
    vertices_trans = transform * vertices_homo;
    p_trans = p;
    p_trans.Vertices = vertices_trans(1:2, :)';
end
