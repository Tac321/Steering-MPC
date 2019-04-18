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

function [ppcx,ppcy]=computeCenter(ppx,ppy,Xtr,Ytr,~)
% Compute the spline approximation of the center of the track.
%
% Works by subdividing every two intervals, splining that interval, and
% defining a new spline point as the point closest to the splined
% subinterval.
%
% Modified by Samuel Zhao Oct.18,2012 so borders is same length as traj
% This function assumes circular end-condition for track.


nbreaks=numel(ppx.breaks); % 667
Xtrs=[Xtr(1:end) Xtr(end)]; % wrap-around, numel(".") = 667
Ytrs=[Ytr(1:end) Ytr(end)]; % wrap-around

% Center track coordinates
Xj=ppval(ppx,ppx.breaks); % Here is the actual cooresponding intertial frame positikons of the center track!
Yj=ppval(ppy,ppx.breaks);


% Below is the normalized length of the track, by spline length
t=ppx.breaks;
t(1)=ppx.breaks(1); %enforce start and end points are same, for wrap around.. Lol this cod edoes nothing. it's redundant.
t(end)=ppx.breaks(end);

% We analyticalize the discrete border track trajectory.
ppcx=spline(t,Xtrs); % Form a spline of the border of center track
ppcy=spline(t,Ytrs);
end


