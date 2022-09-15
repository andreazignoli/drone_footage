# Insights in road cycling downhill performance using aerial drone footages and an 'optimal' reference trajectory

👋 Hi, welcome to the project repository. 

## Synopsis

A new methodology to study performance in downhill road cycling is presented here. A drone was used to capture the trajectory of the centre of mass of a cyclist, who was asked to complete 10 times a ~220-meter-long downhill course. Motion capture trajectories were compared to a theoretical *optimal* trajectory in terms of displacement, speed, and heading. In each trial, the *apex*, the *turn-in* and the *braking points* were detected. Whilst the *optimal* trajectory suggested an *early* apex strategy was best, the cyclist in this study completed the corners with a *late* apex strategy. Therefore, this study presents a methodology that can be used to objectively assess cornering strategies in road cycling.

## Data availability

The code used for the motion capture position detection is of course provided with this repository. Additionally, you can find the files of the 🚁 videos at this [link](https://drive.google.com/drive/folders/1z8yeZPSoOn0l1lVK-wkxGGMYXhY3we03?usp=sharing). 

<p align="center">
<img src="https://github.com/andreazignoli/drone_footage/blob/master/pic/rotated.gif" width="400" height="650"/>
<img src="https://github.com/andreazignoli/drone_footage/blob/master/pic/comparison.png" width="400"/>
</p>

## Mathematical model of bike-rider dynamics

The bike-rider model includes a number of differential equations. The reader is invited to read the 'Additional reading' papers (links below) for definition of the symbols and of the parameters.

The equation of motion for the heading angle is:

![alpha](https://latex.codecogs.com/svg.latex?\frac{d}{ds}\alpha(s)=\frac{\delta_n(s)\delta_{max}}{L\dot{s}(s)}-\kappa(s)) 

The equation of motion for the lateral displacement is:

![n](https://latex.codecogs.com/svg.latex?\frac{d}{ds}n(s)=\frac{1}{\dot{s}(s)}\left(v(s)\sin(\alpha(s))\right)) 

The equation of motion for the roll angle is:

![phi](https://latex.codecogs.com/svg.latex?\frac{d}{ds}\phi(s)=\frac{\dot{\phi}(s)}{\dot{s}(s)})

The equation of motion for the normalized steering angle is:

![delta_n](https://latex.codecogs.com/svg.latex?\frac{d}{ds}\delta_n(s)=\frac{\dot{\delta}_n(s)}{\dot{s}(s)})

The equation of motion for the time is:

![time](https://latex.codecogs.com/svg.latex?\frac{d}{ds}t(s)=-\frac{1+\kappa(s)n(s)}{\cos(\alpha(s))v(s)})

The equation of motion for the time-derivative of the curvilinear abscissa is:

![s_dot](https://latex.codecogs.com/svg.latex?\frac{d}{ds}\dot{s}(s)=\frac{v(s)\cos(\alpha(s))}{1-n(s)\kappa(s)})

The equation of motion for the time-derivative of the roll angle is:

![phi_dot](https://latex.codecogs.com/svg.latex?\frac{d}{ds}\dot{\phi}(s)=\frac{hmg}{I_XgL\dot{s}(s)}(v(s)^2\delta_{max}\delta_n(s)+Lg\phi(s)))

The equation of motion for the normalized power output is:

![W_n](https://latex.codecogs.com/svg.latex?\frac{d}{ds}W_n(s)=\frac{\dot{W}_n(s)}{\dot{s}(s)})

The equation of motion for the longitudinal speed is:

![v](https://latex.codecogs.com/svg.latex?\tiny\frac{mv(s)}{W_{max}}\frac{d}{ds}v(s)=\frac{W_n(s)}{\dot{s}(s)}-\frac{v(s)}{\dot{s}(s)W_{max}}(mg[C_{rr}\cos(\beta(s))+\sin(\beta(s))]+k_v(v(s)-V_w(\alpha(s)))^2))

## Additional reading

In the following papers, reference values and definitions of all parameters and variables are provided. Additionally, more details are provided around the rationale and possible applications of related technologies. 

* [Prediction of pacing and cornering strategies during cycling individual time trials with optimal control](https://link.springer.com/article/10.1007/s12283-020-00326-x)

* [Influence of corners and road conditions on cycling individual time trial performance and ‘optimal’pacing strategy: A simulation study](https://journals.sagepub.com/doi/abs/10.1177/1754337120974872)

* [An intelligent curve warning system for road cycling races](https://link.springer.com/article/10.1007/s12283-021-00356-z)

* [Assessment of bike handling during cycling individual time trials with a novel analytical technique adapted from motorcycle racing](https://www.tandfonline.com/doi/full/10.1080/17461391.2021.1966517)

* [Notes on bike handling in road cycling](https://andreazignoli.github.io/blog-post-4/) - blog article

* [Verona ITT: the perfect combination of power output and bike handling abilities](https://rpubs.com/andrea_zignoli/verona_ITT_2019_vs_2022) - blog article

# Disclosure statement

This study was conducted at the best of our abilities and intentions, with scientific rigour and integrity. 

🙏 Please consider accidental any error included in the code provided with this repository or any misinterpretation of the findings included in the manuscript. 

<p align="center">
<img src="https://github.com/andreazignoli/drone_footage/blob/master/pic/DJI_0132.JPG" width="400"/>
</p>

# Contributors

* [Andrea Zignoli](https://webapps.unitn.it/du/en/Persona/PER0025348/Pubblicazioni)
* [Damiano Fruet](https://webapps.unitn.it/du/en/Persona/PER0049442/Pubblicazioni)