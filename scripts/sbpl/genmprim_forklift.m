% /*
%  * Copyright (c) 2008, Maxim Likhachev
%  * All rights reserved.
%  * 
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions are met:
%  * 
%  *     * Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions and the following disclaimer.
%  *     * Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions and the following disclaimer in the
%  *       documentation and/or other materials provided with the distribution.
%  *     * Neither the name of the University of Pennsylvania nor the names of its
%  *       contributors may be used to endorse or promote products derived from
%  *       this software without specific prior written permission.
%  * 
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  */
function[] = genmprim_forklift(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%
%motion primitives for forklift (marion project)
% 
%written by Ronny Hartanto 
% version with 12 primitives

%defines

UNICYCLE_MPRIM_16DEGS = 1;


if UNICYCLE_MPRIM_16DEGS == 1
    resolution = 0.1; %0.025;
    numberofangles = 16; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 13;%12;%5

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 1;
    backwardcostmult = 2;%1;%4;%5;%10;%5;
    forwardandturncostmult = 2;
    forwardandturncostmult2 = 3;    
    backwardandturncostmult = 3;%2;%5;%6;%20;
    backwardandturncostmult2 = 4;%3;%6;%6;%20;    
    sidestepcostmult = 10;
    turninplacecostmult = 5;%6;%10;%5;
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts0_c(1,:) = [5 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [30 0 0 forwardcostmult];
    basemprimendpts0_c(3,:) = [-5 0 0 backwardcostmult];    
    %1/16 theta change
    basemprimendpts0_c(4,:) = [26 5 1 forwardandturncostmult];
    basemprimendpts0_c(5,:) = [26 -5 -1 forwardandturncostmult];
    % backward
    basemprimendpts0_c(6,:) = [-26 5 -1 backwardandturncostmult];
    basemprimendpts0_c(7,:) = [-26 -5 1 backwardandturncostmult];
    %3/16 theta change
    basemprimendpts0_c(8,:) = [12 6 3 forwardandturncostmult2];
    basemprimendpts0_c(9,:) = [12 -6 -3 forwardandturncostmult2];  
    % backward
    basemprimendpts0_c(10,:) = [-12 6 -3 backwardandturncostmult2];
    basemprimendpts0_c(11,:) = [-12 -6 3 backwardandturncostmult2];    
    %turn in place
    basemprimendpts0_c(12,:) = [0 0 1 turninplacecostmult];  
    basemprimendpts0_c(13,:) = [0 0 -1 turninplacecostmult]; 
    
    %45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts45_c(1,:) = [4 4 0 forwardcostmult];
    basemprimendpts45_c(2,:) = [22 22 0 forwardcostmult];
    basemprimendpts45_c(3,:) = [-4 -4 0 backwardcostmult];    
    %1/16 theta change
    basemprimendpts45_c(4,:) = [15 22 1 forwardandturncostmult];
    basemprimendpts45_c(5,:) = [22 15 -1 forwardandturncostmult];
    % backward
    basemprimendpts45_c(6,:) = [-22 -15 -1 backwardandturncostmult];
    basemprimendpts45_c(7,:) = [-15 -22 1 backwardandturncostmult];
    %3/16 theta change
    basemprimendpts45_c(8,:) = [5 13 3 forwardandturncostmult2];
    basemprimendpts45_c(9,:) = [13 5 -3 forwardandturncostmult2];  
    % backward
    basemprimendpts45_c(10,:) = [-13 -5 -3 backwardandturncostmult2];
    basemprimendpts45_c(11,:) = [-5 -12 3 backwardandturncostmult2];    
    %turn in place
    basemprimendpts45_c(12,:) = [0 0 1 turninplacecostmult];  
    basemprimendpts45_c(13,:) = [0 0 -1 turninplacecostmult];  
    
    %22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts22p5_c(1,:) = [5 2 0 forwardcostmult];
    basemprimendpts22p5_c(2,:) = [28 12 0 forwardcostmult];
    basemprimendpts22p5_c(3,:) = [-5 -2 0 backwardcostmult];    
    %1/16 theta change
    basemprimendpts22p5_c(4,:) = [22 14 1 forwardandturncostmult];
    basemprimendpts22p5_c(5,:) = [26 6 -1 forwardandturncostmult];
    % backward
    basemprimendpts22p5_c(6,:) = [-26 -6 -1 backwardandturncostmult];
    basemprimendpts22p5_c(7,:) = [-22 -14 1 backwardandturncostmult];
    %3/16 theta change
    basemprimendpts22p5_c(8,:) = [9 10 3 forwardandturncostmult2];
    basemprimendpts22p5_c(9,:) = [14 -1 -3 forwardandturncostmult2];  
    % backward
    basemprimendpts22p5_c(10,:) = [-14 1 -3 backwardandturncostmult2];
    basemprimendpts22p5_c(11,:) = [-9 -10 3 backwardandturncostmult2];    
    %turn in place
    basemprimendpts22p5_c(12,:) = [0 0 1 turninplacecostmult];
    basemprimendpts22p5_c(13,:) = [0 0 -1 turninplacecostmult];  
        
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%ColorSet = varycolor(numberofangles);

figure(1);
%iterate over angles
for angleind = 1:numberofangles
    
    %figure(1);
    %hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c = basemprimendpts0_c(primind,:);    
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)
            basemprimendpts_c = basemprimendpts45_c(primind,:);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-7875, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts33p75_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts33p75_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts33p75_c(primind, 3); %reverse the angle as well
            angle = currentangle - 78.75*pi/180;
            fprintf(1, '78p75\n');
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
            %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n');            
        elseif (rem(currentangle_36000int-5625, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts11p25_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts11p25_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts11p25_c(primind, 3); %reverse the angle as well
            angle = currentangle - 56.25*pi/180;
            fprintf(1, '56p25\n');
        elseif (rem(currentangle_36000int-3375, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            angle = currentangle - 33.75*pi/180;
            fprintf(1, '33p75\n');
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        elseif (rem(currentangle_36000int-1125, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            angle = currentangle - 11.25*pi/180;
            fprintf(1, '11p25\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        %now figure out what action will be        
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
            %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        end;
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 10;
        intermcells_m = zeros(numofsamples,3);
        if UNICYCLE_MPRIM_16DEGS == 1
            startpt = [0 0 currentangle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            if ((endx_c == 0 & endy_c == 0) | baseendpose_c(3) == 0) %turn in place or move forward            
                for iind = 1:numofsamples
                    intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                            startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                            rem(startpt(3) + (endpt(3) - startpt(3))*(iind-1)/(numofsamples-1), 2*pi)];
                end;   
                
                
                         
            else %unicycle-based move forward or backward
                R = [cos(startpt(3)) sin(endpt(3)) - sin(startpt(3));
                    sin(startpt(3)) -(cos(endpt(3)) - cos(startpt(3)))];
                S = pinv(R)*[endpt(1) - startpt(1); endpt(2) - startpt(2)];
                l = S(1); 
                tvoverrv = S(2);
                rv = (baseendpose_c(3)*2*pi/numberofangles + l/tvoverrv);
                tv = tvoverrv*rv;
                         
                if ((l < 0 & tv > 0) | (l > 0 & tv < 0))
                    fprintf(1, 'WARNING: l = %d < 0 -> bad action start/end points\n', l);
                    l = 0;
                end;
                %compute rv
                %rv = baseendpose_c(3)*2*pi/numberofangles;
                %compute tv
                %tvx = (endpt(1) - startpt(1))*rv/(sin(endpt(3)) - sin(startpt(3)))
                %tvy = -(endpt(2) - startpt(2))*rv/(cos(endpt(3)) - cos(startpt(3)))
                %tv = (tvx + tvy)/2.0;              
                %generate samples
                for iind = 1:numofsamples                                        
                    dt = (iind-1)/(numofsamples-1);
                                        
                    %dtheta = rv*dt + startpt(3);
                    %intermcells_m(iind,:) = [startpt(1) + tv/rv*(sin(dtheta) - sin(startpt(3))) ...
                    %                        startpt(2) - tv/rv*(cos(dtheta) - cos(startpt(3))) ...
                    %                        dtheta];
                    
                    if(abs(dt*tv) < abs(l))
                        intermcells_m(iind,:) = [startpt(1) + dt*tv*cos(startpt(3)) ...
                                                 startpt(2) + dt*tv*sin(startpt(3)) ...
                                                 startpt(3)];
                    else
                        dtheta = rv*(dt - l/tv) + startpt(3);
                        intermcells_m(iind,:) = [startpt(1) + l*cos(startpt(3)) + tvoverrv*(sin(dtheta) - sin(startpt(3))) ...
                                                 startpt(2) + l*sin(startpt(3)) - tvoverrv*(cos(dtheta) - cos(startpt(3))) ...
                                                 dtheta];
                    end;
                end; 
                %correct
                errorxy = [endpt(1) - intermcells_m(numofsamples,1) ... 
                           endpt(2) - intermcells_m(numofsamples,2)];
                fprintf(1, 'l=%f errx=%f erry=%f\n', l, errorxy(1), errorxy(2));
                interpfactor = [0:1/(numofsamples-1):1];
                intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
                intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
            end;                                        
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        %% original plot commands
        %plot(intermcells_m(:,1), intermcells_m(:,2));
        %axis([-1.2 1.2 -1.2 1.2]);
        %text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        %hold on;
        
        %% plot commands for presentation
        %xlabel('y(m)');
        %ylabel('x(m)');
        %plot(intermcells_m(:,1), intermcells_m(:,2),'Linewidth',1.25);
        %axis([-1.5 1.75 -1 1]);
        %axis equal;
        %text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)), 'FontSize', 12);
        %view(-90,90)
        %hold on;
        
        %% plot commands for presentation with all angle
        xlabel('x(m)');
        ylabel('y(m)');
        plot(intermcells_m(:,1), intermcells_m(:,2),'Linewidth',1.5, 'Color', 'black');
        axis([-2.22 2.22 -2.22 2.22]);
        axis manual;
        axis equal;
        grid;
        %text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)), 'FontSize', 12);
        hold on;
        
    end;
    grid;
    pause;
end;
hold off;
        
fclose('all');
