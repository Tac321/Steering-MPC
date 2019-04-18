% Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% track Is the new virtual 'safety' track we've created
function [track,track2] = borderAdjustment(track2,ModelParams,safetyScaling)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% shrink track on both sides with 'trackShrinkPerc' %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trackShrinkPerc = 0.5*ModelParams.W*safetyScaling;  % Half the width of the car.
%scale track to car size, OMG Nice!
track2.center = track2.center*ModelParams.Scale;   % Scale == 1 for the R/C car
track2.inner = track2.inner*ModelParams.Scale;
track2.outer = track2.outer*ModelParams.Scale;

% compute width of track (assumption track as a uniform width
%   Note: Above is used just to draw the virtual boundary track of which
%   truly drives our car!
widthTrack = norm([track2.inner(1,1)-track2.outer(1,1),track2.inner(2,1)-track2.outer(2,1)]); % Norm(Dx, Dy)

for i = 1:length(track2.outer)
    x1 = track2.outer(1,i);
    y1 = track2.outer(2,i);
    x2 = track2.inner(1,i);
    y2 = track2.inner(2,i);
    % vector connecting right and left boundary
    numer=(x2-x1);   % ?
    denom=(y1-y2);
    
    % shrinking ratio
    c =  trackShrinkPerc/widthTrack; % (Half width of the car)/(width of track)
    d = -trackShrinkPerc/widthTrack;
    
    
    % Note: Because of the definitions of "left" and right" tracks this
    % would work for shrinking any track even an infinity signed track!
    % shrink track
    % Construct a new struct real time.
    track.outer(1,i) = x1 + c*numer;  % The right track
    track.inner(1,i) = x2 - c*numer;  % The left track
    track.outer(2,i) = y1 + d*denom;  % The right track
    track.inner(2,i) = y2 - d*denom;  % The left track
end
track.center = track2.center;

end












