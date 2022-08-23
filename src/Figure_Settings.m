%% Set the figure
% docked figure style
set(0, 'DefaultFigureWindowStyle', 'normal')
% bold font style
set(0, 'DefaultLineLineWidth', 1); %, 'DefaultTextFontWeight', 'bold')
% Times New Roman font style
set(0, 'DefaultAxesFontName', 'Brandon Grotesque')
% 14 font size
set(0, 'DefaultAxesFontSize', 16,'DefaultTextFontSize',14)
% white color background
set(0, 'defaultfigurecolor', [1 1 1])
% marker size 10
set(0, 'DefaultLineMarkerSize', 8);

%% color options
% red
opts_r = {'EdgeColor', 'none','FaceAlpha',.3,...
    'FaceColor', [1 0.5 0.5]};
% blue
opts_b = {'EdgeColor', 'none','FaceAlpha',.3,...
    'FaceColor', [0.5 0.5 1]};
% black
opts_k = {'EdgeColor', 'none','FaceAlpha',.3,...
    'FaceColor', [.5 .5 .5]};
% green
opts_g = {'EdgeColor', 'none','FaceAlpha',.3,...
    'FaceColor', [.5 1 .5]};

%% color maps
redMap   = [linspace(1,0,256)', ones(256,2)];
greenMap = [zeros(256,1), linspace(0,1,256)', zeros(256,1)];
blueMap  = [zeros(256,2), linspace(0,1,256)'];

%% color codes
colSilver          = [192,192,192]/255;
colSlateGray       = [112,128,144]/255;
colLightSlateGray  = [119,136,153]/255;

colDarkYellow      = [245,208,15]/255;

colGreen           = [0,255,0]/255;
colRed             = [255,0,0]/255;
colBlue            = [0,0,255]/255;
colNavy            = [0,0,128]/255;
colSteelBlue       = [70,130,180]/255;
colCornFlowerBlue  = [100,149,237]/255;
colDodgerBlue      = [30,144,255]/255;
colSkyBlue         = [135,206,235]/255;
colLightSteelBlue  = [176,196,222]/255;
colBrown           = [165,42,42]/255;
colDarkBlue        = [0,0,139]/255;

colLime         = [0,255,0]/255;
colLimeGreen    = [50,205,50]/255;
colLawnGreen    = [124,252,0]/255;
colGreenYellow  = [173,255,47]/255;
colDarkGreen    = [0,100,0]/255;
colForestGreen  = [34,139,34]/255;
colOlive        = [128,128,0]/255;

colPurple       = [128,0,128]/255;
colViolet       = [238,130,238]/255;
colOrchid       = [218,112,214]/255;
colDeepPink     = [255,20,147]/255;

colOrangeRed    = [255,69,0]/255;
colDarkOrange   = [255,140,0]/255;
colOrange       = [255,165,0]/255;
colGoldenRod    = [218,165,32]/255;
colSpringGreen  = [0,255,127]/255;
