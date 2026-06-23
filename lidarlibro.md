# lidarlibro

> Conversión automática a Markdown desde el PDF original. Se conserva el texto extraído por página y se añaden las imágenes extraídas como recursos cuando están disponibles.

---

## Página 1

traction of individual local features while still able to identify robustly the correct matching string in a variety of circumstances. It should be clear to the reader that the image histogram and image fingerprint place representations are straightforward to implement. For this reason these methods became very popular, although recently the visual-word-based approaches for a greater variety of applications have outperformed them.

## 4.7 Feature Extraction Based on Range Data (Laser, Ultrasonic)

Most of today's features extracted from ranging sensors are geometric primitives such as line segments or circles. The main reason for this is that for most other geometric primitives the parametric description of the features becomes too complex and no closed-form solution exists. In this section, we will focus on line extraction, since line segments are the simplest features to extract. As we will see in chapter 5, lines are used to match laser scans for performing tasks like robot localization or automatic map building. There are three main problems in line extraction in unknown environments:

- How many lines are there?

- Which points belong to which line?

- Given the points that belong to a line, how to estimate the line model parameters?

For answering these questions, we will present the description of the six most popular line extraction algorithms for 2D range scans. Our selection is based on their performance and popularity in both mobile robotics, especially feature extraction, and computer vision. Only basic versions of the algorithms are given, even though their details may vary in different applications and implementations. The interested reader should refer to the indicated

**Figure 4.87**

Three actual string sequences. The top two are strings extracted by the robot at the same position [182].

### Imágenes extraídas de esta página

![Imagen extraída página 1](assets/page_01_img_01.png)

![Imagen extraída página 1](assets/page_01_img_02.png)

---

## Página 2

references for more details. Our implementation follows closely the pseudocode described below in most cases, otherwise it will be stated. Before describing the six algorithms, we will first explain the line fitting problem, which answers the third question: ³Given the points that belong to a line, how to estimate the line model parameters?´ In describing line fitting, we will demonstrate how the uncertainty models presented in section 4.1.3 can be applied to the problem of combining multiple sensor measurements. Then, we will answer the first two questions by describing six lineextraction algorithms from noisy range measurements. Finally, we will briefly present other very successful features of indoor mobile robots using range data, the corner and the plane features, and demonstrate how these features can be combined into a single representation.

### 4.7.1 Line fitting

Geometric feature fitting is usually the process of comparing and matching measured sensor data against a predefined description, or template, of the expected feature. Usually, the system is overdetermined in that the number of sensor measurements exceeds the number of feature parameters to be estimated. Since the sensor measurements all have some error, there is no perfectly consistent solution and, instead, the problem is one of optimization. One can, for example, fit the feature that minimizes the discrepancy with all sensor measurements used (e.g,. least-squares estimation). In this section we present an optimization solution to the problem of extracting a line feature from a set of uncertain sensor measurements. For greater detail than what is presented below, refer to [17, pages 15 and 221].

#### 4.7.1.1 Probabilistic line fitting from uncertain range sensor data

Our goal is to fit a line to a set of sensor measurements as shown in figure 4.88. There is uncertainty associated with each of the noisy range sensor measurements, and so there is no single line that passes through the set. Instead, we wish to select the best possible match, given some optimization criterion. More formally, suppose ranging measurement points in polar coordinates are produced by the robot's sensors. We know that there is uncertainty associated with each measurement, and so we can model each measurement using two random variables . In this analysis we assume that the uncertainty with respect to the actual value of and is independent. Based on equation (4.11), we can state this formally: = . (4.125) = . (4.126) n xi i i = Xi Pi Qi = P Q E Pi Pj E Pi E Pj i j n i j = E Qi Qj E Qi E Qj i j n  i j =

---

## Página 3

= . (4.127) Furthermore, we assume that each random variable is subject to a Gaussian probability density curve, with a mean at the true value and with some specified variance: ~ . (4.128) ~ . (4.129) Given some measurement point , we can calculate the corresponding Euclidean coordinates as and . If there were no error, we would want to find a line for which all measurements lie on that line: . (4.130) Of course, there is measurement error, and so this quantity will not be zero. When it is nonzero, this is a measure of the error between the measurement point and the line, specifically in terms of the minimum orthogonal distance between the point and the line. It is always important to understand how the error that shall be minimized is being measured. For example, a number of line-extraction techniques do not minimize this orthogonal pointline distance, but instead the distance parallel to the y-axis between the point and the line. A good illustration of the variety of optimization criteria is available in [25], where several r

**Figure 4.88**

Estimating a line in the least-squares sense. The model parameters r (length of the perpendicular) and (its angle to the abscissa) uniquely describe a line. di xi = ( i, i) E Pi Qj E Pi E Qj i j n = Pi N i i Qi N i i x cos = y sin = cos cos sin sin r ± + ± cos r ± = =

---

## Página 4

algorithms for fitting circles and ellipses are presented that minimize algebraic and geometric distances. For each specific , we can write the orthogonal distance between and the line as . (4.131) If we consider each measurement to be equally uncertain, we can sum the square of all errors together, for all measurement points, to quantify an overall fit between the line and all of the measurements: . (4.132) Our goal is to minimize when selecting the line parameters . We can do so by solving the nonlinear equation system . (4.133) This formalism is considered an unweighted least-squares solution because no distinction is made from among the measurements. In reality, each sensor measurement may have its own, unique uncertainty based on the geometry of the robot and environment when the measurement was recorded. For example, we know with regard to vision stereo ranging that uncertainty and, therefore, variance increase as a square of the distance between the robot and the object. To make use of the variance that models the uncertainty regarding distance of a particular sensor measurement, we compute an individual weight for each measurement using the formula17 . (4.134) Then, equation (4.132) becomes . (4.135)

17.The issue of determining an adequate weight when
is given (and perhaps some additional information) is complex in general and beyond the scope of this text. See [11] for a careful treatment. i i di i i i i ± cos r ± di = S di i i i ± cos r ± i = = S r S = r S = i i wi i wi i = S widi wi i i ± cos r ± = =

---

## Página 5

It can be shown that the solution to equation (4.133) in the weighted least-squares sense18 is . (4.136) . (4.137) In practice, equation (4.136) uses the four-quadrant arc tangent (atan2).19 Let us demonstrate equations (4.136) and (4.137) with a concrete example. The seventeen measurements in table 4.3 have been taken with a laser range sensor installed on a mobile robot. The measurements are shown in figure 4.89. The measurement uncertainty is usually considered proportional to the measured distance, but, to simplify the calculation, in this case we assume that the uncertainties of all measurements are equal. We also assume that the measurements are uncorrelated, and that the robot was static during the measurement process. Direct application of this solution equations yields the line defined by and . This line represents the best fit in a least-squares sense and is shown visually in figure 4.89.

#### 4.7.1.2 Propagation of uncertainty during line fitting

Returning to the subject of section 4.1.3, we would like to understand how the uncertainties of specific range sensor measurements propagate to govern the uncertainty of the extracted line. In other words, how does uncertainty in and propagate in equations (4.136) and (4.137) to affect the uncertainty of and ? This requires direct application of equation (4.15) with and representing the random output variables of and  respectively. The goal is to derive the output covariance matrix

18.We follow here the notation of [17] and distinguish a weighted least-squares problem if
is diagonal (input errors are mutually independent) and a generalized least-squares problem if is non-diagonal.

19.Atan2 computes
but uses the signs of both x and y to determine the quadrant in which the resulting angles lies. For example , whereas , a distinction which would be lost with a single-argument arc tangent function. CX CX 2--atan wi i i sin wi ------- wiwj i j i cos j sin ± wi i i cos wi ------- wiwj i j i j + cos ± ------------------------------------------------------------------------------------------------------------------- = r wi i i ± cos wi --------------------------------------------- = x y tan ± ± ± atan ± =

2 2 2
atan = i i

## 37.36 =

r

## 0.4 =

i i r A R r

---

## Página 6

**Table 4.3 Measured values**

pointing angle of sensor i [deg] range i [m]

0.5197
0.4404
0.4850
0.4222
0.4132
0.4371
0.3912
0.3949
0.3919
0.4276
0.4075
0.3956
0.4053
0.4752
0.5032
0.5273
## 0.4879 Figure 4.89

Extracted line from laser range measurements (+). The small lines at each measurement point represent the measurement uncertainty  that is proportional to the measured distance. x y

### Imágenes extraídas de esta página

![Imagen extraída página 6](assets/page_06_img_01.png)

![Imagen extraída página 6](assets/page_06_img_02.png)

![Imagen extraída página 6](assets/page_06_img_03.png)

---

## Página 7

, (4.138) given the input covariance matrix (4.139) and the system relationships [equations (4.136) and (4.137)]. Then by calculating the Jacobian, , (4.140) we can instantiate the uncertainty propagation equation (4.15) to yield : (4.141) Thus we have calculated the probability of the extracted line based on the probabilities of the measurement points. For more details about this method, refer to [8, 59].

### 4.7.2 Six line-extraction algorithms

The previous section described how to fit a line feature given a set of range measurements. Unfortunately, the feature extraction process is significantly more complex than that. A mobile robot does indeed acquire a set of range measurements, but in general the range measurements are not all part of one line. Rather, only some of the range measurements should play a role in line extraction and, further, there may be more than one line feature represented in the measurement set. This more realistic scenario is shown in figure 4.90. The process of dividing up a set of measurements into subsets that can be interpreted one by one is termed segmentation and is the most important step of line extraction. In the fol- CAR A AR AR R = 2n 2n CX CP

0 CQ
diag i diag i = = FPQ P1 P2 Pn Q1 Q2 Qn P1 r P2 r Pn r Q1 r Q2 r Qn r = CAR CAR FPQCXFPQ T = CAR r

---

## Página 8

lowing, we describe six popular line-extraction (segmentation) algorithms. For both an overview and a comparison among these algorithms, we refer the reader to [247].

#### 4.7.2.1 Algorithm 1: Split-and-merge

Split-and-Merge is the most popular line extraction algorithm. This algorithm has originated from computer vision [257] and has been studied and used in many works [96, 121, 287, 78, 336]. The algorithm is outlined in algorithm 1. Notice that this algorithm can be slightly modified on line 3 to make it more robust to noise. Indeed, sometimes the splitting position can be the result of a point which still belongs to the same line but which, because of noise, appears too far away from this line.

**Figure 4.90**

Clustering: finding neighboring segments of a common line [59]. 1=r [m] A set of nf neighboring points of the image space Evidence accumulation in the model space Clusters of normally distributed vectors (a) Image Space (b) Model Space 0= [rad]

### Algorithm 1: Split-and-Merge

1. Initial: set
consists of N points. Put in a list L

2. Fit a line to the next set
in L

3. Detect point P with maximum distance
to the line

4. If
is less than a threshold, continue (go to step 2)

5. Otherwise, split
at P into and , replace in L by and , continue (go to 2)

6. When all sets (segments) in L have been checked, merge collinear segments.
s1 s1 si dP dP si si1 si2 si si1 si2

---

## Página 9

In this case, we scan for a splitting position where two adjacent points and are on the same side of the line and both have distances to the line greater than the threshold. If we find only one such point, then we automatically discard it as a noisy point. Observe that in line 2 one can use for line fitting the least-squares method described in section 4.7.1. Alternatively, one can construct the line by simply connecting the first and the last points. In this case, the algorithm is named Iterative-End-Point-Fit [19, 287, 78, 336]   and is a well consolidated approach to implement split-and-merge. This procedure is illustrated in figure 4.91. Finally, an application of split-and-merge to a 2D laser scan is shown in figure 4.92.

#### 4.7.2.2 Algorithm 2: Line regression

This algorithm was proposed in [59]. It uses a sliding window of size . At every step, a line is fitted to the points within the window. The window is then shifted one point forward (this is why it is called sliding window), and the line-fitting operation is repeated again. The goal is to find adjacent line segments and merge them together. To do this, at every step the Mahalanobis20 distance between the last two windows is computed and is stored in a fidelity array. When all the points have been analyzed, the fidelity array is scanned for consecutive similar elements. This is done by using an appropriate clustering

20.The Mahalanobis distance is defined in section ³Matching´ on page 334.
P1 P2

**Figure 4.91 Split-and-merge implemented in the Iterative-End-Point-Fit fashion. In this case, the line**

is not fitted to the points but is constructed by connecting the first and last points. Nf Nf

### Imágenes extraídas de esta página

![Imagen extraída página 9](assets/page_09_img_01.png)

![Imagen extraída página 9](assets/page_09_img_02.png)

![Imagen extraída página 9](assets/page_09_img_03.png)

![Imagen extraída página 9](assets/page_09_img_04.png)

![Imagen extraída página 9](assets/page_09_img_05.png)

![Imagen extraída página 9](assets/page_09_img_06.png)

![Imagen extraída página 9](assets/page_09_img_07.png)

---

## Página 10

algorithm. At the end, the clustered consecutive line segments are merged together using again line regression. This algorithm is outlined in algorithm 2, while the main steps are depicted in figure 4.93. Notice that the sliding window size is very dependent on the environment and has a strong influence on the algorithm performance. In typical applications, = 7 is used.

#### 4.7.2.3 Algorithm 3: Incremental

This algorithm is straightforward to implement and has been used in many applications [24, 328, 308]. The algorithm is outlined in algorithm 3. At the beginning, the set consists of

**Figure 4.92 Split-and-merge applied to a 2D laser scan (courtesy of B. Jensen).**

Nf Nf

### Algorithm 2: Line-Regression

1. Initialize sliding window size
2. Fit a line to every
consecutive points

3. Compute a line fidelity array. Each element of the array contains the sum of Mahalanobis
distances between every three adjacent windows

4. Construct line segments by scanning the fidelity array for consecutive elements having values less than a threshold
5. Merge overlapped line segments and recompute line parameters for each segment
Nf Nf

### Imágenes extraídas de esta página

![Imagen extraída página 10](assets/page_10_img_01.png)

![Imagen extraída página 10](assets/page_10_img_02.png)

---

## Página 11

ments may contain distinct lines from the surrounding walls but also points from other static and dynamic objects (like chairs or humans). In this case, an outlier is any entity which does not belong to a line (i.e., chair, human, and so on). RANSAC is an iterative method and is nondeterministic in that the probability to find a line free of outliers increases as more iterations are used. RANSAC is not restricted to line extraction from laser data but it can be generally applied to any problem where the goal is to identify the inliers which satisfy a predefined mathematical model. Typical applications in robotics are: line extraction from 2D range data (sonar or laser); plane extraction from 3D laser point clouds; and structure-from-motion (section 4.2.6), where the goal is to identify the image correspondences which satisfy a rigid body transformation. Let us see how RANSAC works for the simple case of line extraction from 2D laser scan points. The algorithm starts by randomly selecting a sample of two points from the dataset. Then a line is constructed from these two points and the distance of all other points to this line is computed. The inliers set comprises all the points whose distance to the line is within a predefined threshold d. The algorithm then stores the inliers set and starts again by selecting another minimal set of two points at random. The procedure is iterated until a set with a maximum number of inliers is found, which is chosen as a solution to the problem. The algorithm is outlined in algorithm 4, while figure 4.94 illustrates its working principle. Because we cannot know in advance if the observed set contains the maximum number of inliers, the ideal would be to check all possible combinations of 2 points in a dataset of N points. The number of combinations is given by , which makes it computationally unfeasible if N is too large. For example, in a laser scan of 360 points we would need to check all = 64,620 possibilities! At this point, a question arises: Do we really need to check all possibilities, or can we stop RANSAC after  iterations? The answer is that indeed we do not need to check all combinations but just a subset of them if we have a rough estimate of the percentage of inliers in our dataset. This can be done by thinking in a probabilistic way.

### Algorithm 3: Incremental

1. Start by the first 2 points, construct a line
2. Add the next point to the current line model
3. Recompute the line parameters by line fitting
4. If it satisfies the line condition, continue (go to step 2)
5. Otherwise, put back the last point, recompute the line parameters, return the line
6. Continue with the next two points, go to step 2
N N ±

360 359 2
k

---

## Página 12

Let  be the probability of finding a set of points free of outliers. Let w be the probability of selecting an inlier from our dataset of N points. Hence, w expresses the fraction of inliers in the data, that is, = number of inliers/N. If we assume that the two points needed for estimating a line are selected independently, is the probability that both points are inliers and is the probability that at least one of these two points is an outlier. Now, let be the number of RANSAC iterations executed so far, then will be the probability that RANSAC never selects two points that are both inliers. This probability must be equal to . Accordingly, , (4.142) and therefore . (4.143) This expression tells us that knowing the fraction of inliers, after  RANSAC iterations we will have a probability  of finding a set of points free of outliers. For example, if we want a probability of success equal to 99% and we know that the percentage of inliers in the dataset is 50%, then according to (4.143) we could stop RANSAC after 16 iterations, which is much less than the number of all possible combinations that we had to check in the previous example! Also observe that in practice we do not need a precise knowledge of

### Algorithm 4: RANSAC

1. Initial: let A be a set of N points
2. repeat
3. Randomly select a sample of 2 points from A
4. Fit a line through the 2 points
5. Compute the distances of all other points to this line
6. Construct the inlier set (i.e. count the number of points with distance to the line < d)
7. Store these inliers
8. until Maximum number of iterations  reached
9. The set with the maximum number of inliers is chosen as a solution to the problem
k p w w2 w ± k w2 ± k p ± p ± w2 ± k = k p ± log w2 ± log ---------------------------- = w k p

---

## Página 13

the fraction of inliers but just a rough estimate. More advanced implementations of RANSAC estimate the fraction of inliers by changing it adaptively iteration after iteration. The main advantage of RANSAC is that it is a generic extraction method and can be used with many types of features once we have the feature model. Because of this, it is very popular in computer vision [29]. It is also simple to implement. Another advantage is its ability to cope with large amount of outliers, even more than 50%. Clearly, if we want to extract multiple lines, we need to run RANSAC several time and remove sequentially all (a) (b) (c) (d)

**Figure 4.94 Working principle of RANSAC. (a) Dataset of N points. (b) Two points are randomly**

selected, a line is fitted through them, and the points within a predefined distance to it are identified. (c) The procedure is repeated (iterated) several times. (d) The set with the maximum number of inliers is chosen as a solution to the problem.

### Imágenes extraídas de esta página

![Imagen extraída página 13](assets/page_13_img_01.png)

![Imagen extraída página 13](assets/page_13_img_02.png)

---

## Página 14

the lines extracted so far. A disadvantage of RANSAC is that when the maximum number of iterations  is reached, the solution obtained may not be the optimal one (i.e., the one with the maximum number of inliers). Furthermore, this solution may not even be the one that fits the data in the best way.

#### 4.7.2.5 Algorithm 5: Hough Transform (HT)

This algorithm was already described for straight edge detection in intensity images (page 205) but it can be applied without any modification to 2D range images. The algorithm is outlined in algorithm 5.   Although it has been developed within the computer vision community, it has been brought into robotics for extracting lines from scan images [158] and [261]. In fact, 2D scan images are nothing but binary images. Typical drawbacks with the Hough transform are that it is usually difficult to choose an appropriate grid size and the fact that this transform does not take noise and uncertainty into account when estimating the line parameters. To overcome the second problem, in line 7 one can use the line fitting method described in section 4.7.1, which takes into account feature uncertainty.

#### 4.7.2.6 Algorithm 6: Expectation maximization (EM)

Expectation Maximization (EM), is a probabilistic method commonly used in missing variable problems. EM has been used as a line extraction tool in computer vision [24] and robotics [261]. There are some drawbacks with the EM algorithm. First, it can fall into local minima. Second, it is difficult to choose a good initial value. The algorithm is outlined in algorithm 6. For a detailed implementation of this algorithm for extracting lines, we refer the reader to [24]. k

### Algorithm 5: Hough Transform

1. Initial: let A be a set of N points
2. Initialize the accumulator array by setting all elements to 0
3. Construct values for the array
4. Choose the element with max. votes
5. If
is less than a threshold, terminate

6. Otherwise, determine the inliers
7. Fit a line through the inliers and store the line
8. Remove the inliers from the set, go to step 2
Vmax Vmax

---

## Página 15

#### 4.7.2.7 Implementation details

Clustering.  In most cases, 2D laser scans present some agglomerations of a few sparse points (figure 4.92). These points can be caused for instance by small objects or moving people. In this case, a simple clustering algorithm is usually used for preprocessing: it divides the raw points into groups of close points and discards groups consisting of too few points. Basically, this algorithm scans for big jumps in radial differences of consecutive points and puts breakpoints in those positions. As a result, the scan is segmented into contiguous clusters of points. Clusters having too few number of points are removed. Merging.  Due to occlusions, a line may be observed and extracted as several segments. When this happens, it is likely good to merge collinear line segments into a single line segment. This merging routine should be applied at the output end of each previously seen algorithm, after segments have been extracted. To decide if two consecutive line segments have to be merged, the Mahalanobis distance21 between each pair of line segments is typically used. If the two line segments have Mahalanobis distance less than a predefined

21.The Mahalanobis distance depends on the covariance matrix of the parameters of each line segment as explained on page 334.
### Algorithm 6: Expectation Maximization

1. Initial: let A be a set of N points
2.   repeat
3.    Randomly generate parameters for a line
4.    Initialize weights for remaining points
5.    repeat
6.      E-Step: Compute the weights of the points from the line model
7.      M-Step: Recompute the line model parameters
8.    until Maximum number of steps reached or convergence
9.   until Maximum number of trials reached or found a line
10. If found, store the line, remove the inliers, go to step 2
11 Otherwise, terminate
---

## Página 16

threshold, then they are merged. Using line fitting, the new line parameters are finally recomputed from the raw scan points that constitute the two segments.

#### 4.7.2.8 A comparison of line extraction algorithms

These six algorithms can be divided into two categories: deterministic and nondeterministic methods:

1. Deterministic: Split-and-Merge, Incremental, Regression, Hough transform.
2. Nondeterministic: RANSAC, EM.
RANSAC and EM are nondeterministic because their results can be different at every run. This is because these two algorithms generate random hypotheses. A comparison between all six algorithms has been done by Nguyen et al. [247]. They evaluated four quality measures: complexity, speed, correctness (false positives), and precision. The results of that study are shown in table 4.4. The terminology used is explained as follows:

- N: Number of points in the input scan (e.g., 722)

- S: Number of line segments extracted (e.g., 7 in average depending on the algorithm)

- : Sliding window size for Line-Regression (e.g., 9)

- : Number of trials for RANSAC (e.g., 1000)

- ,

: Number of columns, rows respectively for the Hough accumulator array ( = 401, = 671 for resolutions of 1 cm and 0.9 degrees)

- ,

: Number of trials and convergence iterations, respectively, for EM (e.g. = 50, = 200). Observe that the values shown in parentheses are typical numbers used in practical implementations. As shown in the third column (Speed) of table 4.4, Split-and-Merge, Incremental, and Line-Regression perform much faster than the others. The Split-and-Merge algorithm takes the lead. The reason why these three algorithms are much faster is mainly because they are deterministic and, especially, because they take advantage of the sequential ordering of the raw scan points (the points are not captured randomly but according to the rotation direction of the laser beam). If these three algorithms were applied on randomly distributed points (e.g., general binary images), they would not be able to segment all lines, while RANSAC, EM, and Hough would. Indeed, these last three algorithms are popular for their ability to extract lines in binary images which obviously present a large number of outliers. The Incremental algorithm seems to perform the best in terms of correctness. In fact, it has a very low number of false positives, which is very important for localization, mapping, Nf NTrials NC NR NC NR N1 N2 N1 N2

---

## Página 17

and SLAM (section 5.8). Conversely, RANSAC, HT, and EM seem to produce many more false positives. This is due to the fact that they do not take advantage of the sequential ordering of the scan points and therefore they often try to fit lines falsely across the scan map. Their behavior could be improved by increasing the minimum number of points per line segment, but the drawback of this would then be that short segments might be left out. Despite their bad correctness, as observed in the fourth column of table 4.4, RANSAC, HT, and EM produce more precise lines than the other algorithms. This is due to their ability to get rid of outliers or largely noisy inliers. For instance, with RANSAC the probability of extracting a stable line increases with the number of iterations, while with HT the outlier (or a largely noise inlier) would vote another grid cell than that representing the line. In conclusion, Split-and-Merge and Incremental are the best choice in terms of correctness and efficiency and are therefore the best candidates for 2D laser-based robot localization and mapping. However, the right choice depends highly on the type of application and the desired precision.

### 4.7.3 Range histogram features

A histogram is a simple way to combine characteristic elements of an image. An angle histogram, as presented in figure 4.95, plots the statistics of lines extracted by two adjacent range measurements. First, a 360-degree scan of the room is taken with the range scanner, and the resulting ³hits´ are recorded in a map. Then the algorithm measures the relative angle between any two adjacent hits (see figure 4.95b). After compensating for noise in the readings (caused by the inaccuracies in position between adjacent hits), the angle histogram

**Table 4.4  Comparison of algorithms for line extraction from 2D laser data.**

Complexity Speed [Hz] False positives Precision Split-and-Merge 10% +++ Incremental 6% +++ Line-Regression 10% +++ RANSAC 30% ++++ Hough-Transform 30% ++++ Expectation Maximization 50% ++++ N N log S N2 N Nf S N NTrials S N NC S NR NC + S N1 N2 N

---

## Página 18

shown in figure 4.95c can be built. The uniform direction of the main walls are clearly visible as peaks in the angle histogram. Detection of peaks yields only two main peaks: one for each pair of parallel walls. This algorithm is very robust with regard to openings in the walls, such as doors and windows, or even cabinets lining the walls.

### 4.7.4 Extracting other geometric features

Line features are of particular value for mobile robots operating in man-made environments, where, for example, building walls and hallway walls are usually straight. In general, a mobile robot makes use of multiple features simultaneously, comprising a feature set that is most appropriate for its operating environment. For indoor mobile robots, the line feature is certainly a member of the optimal feature set. In addition, other geometric kernels consistently appear throughout the indoor manmade environment. Corner features are defined as a point feature with an orientation. Step discontinuities, defined as a step change perpendicular to the direction of hallway travel,

**Figure 4.95**

Angle histogram [329].

### Imágenes extraídas de esta página

![Imagen extraída página 18](assets/page_18_img_01.png)

![Imagen extraída página 18](assets/page_18_img_02.png)

![Imagen extraída página 18](assets/page_18_img_03.png)

![Imagen extraída página 18](assets/page_18_img_04.png)

![Imagen extraída página 18](assets/page_18_img_05.png)

![Imagen extraída página 18](assets/page_18_img_06.png)

---

## Página 19

are characterized by their form (convex or concave) and step size. Doorways, defined as openings of the appropriate dimensions in walls, are characterized by their width. Thus, the standard segmentation problem is not so simple as deciding on a mapping from sensor readings to line segments, but rather it is a process in which features of different types are extracted based on the available sensor measurements. Figure 4.96 shows a model of an indoor hallway environment along with both indentation features (i.e., step discontinuities) and doorways. Note that different feature types can provide quantitatively different information for mobile robot localization. The line feature, for example, provides two degrees of information, angle and distance. But the step feature provides 2D relative position information as well as angle. The set of useful geometric features is essentially unbounded, and as sensor performance improves we can only expect greater success at the feature extraction level. For example, an interesting improvement upon the line feature described above relates to the advent of successful vision ranging systems (e.g., stereo cameras and time-of-flight cameras) and 3D laser rangefinder. Because these sensor modalities provide a full 3D set of range measurements, one can extract plane features in addition to line features from the resulting data set. Plane features are valuable in man-made environments due to the flat walls, floors, and ceilings of our indoor environments. Thus they are promising as another highly informative feature for mobile robots to use for mapping and localization. Some

**Figure 4.96**

Multiple geometric features in a single hallway, including doorways and discontinuities in the width of the hallway.

### Imágenes extraídas de esta página

![Imagen extraída página 19](assets/page_19_img_01.png)

![Imagen extraída página 19](assets/page_19_img_02.png)

![Imagen extraída página 19](assets/page_19_img_03.png)

![Imagen extraída página 19](assets/page_19_img_04.png)

![Imagen extraída página 19](assets/page_19_img_05.png)

---

## Página 20

experiments using plane features have been done at the ASL (ETH Zurich) [331], the plane feature extraction process is illustrated in figure 4.97.

## 4.8 Problems

1. Consider an omnidirectional robot with a ring of eight 70 KHz sonar sensors that are
fired sequentially. Your robot is capable of accelerating and decelerating at 50 cm/ . It is moving in a world filled with sonar-detectable fixed (nonmoving) obstacles that can only be detected at 5 meters and closer. Given the bandwidth of your sonar sensors, compute your robot's appropriate maximum speed to ensure no collisions.

2. Design an optical triangulation system with the best possible resolution for the following
conditions: specify b (as in figure 4.15): (a) the system must have sensitivity of 1 cm at a range of 2 meters. (b) The PSD has a sensitivity of 0.1 mm. (c) f = 10 cm.

**Figure 4.97 Extraction process of plane features: (Upper left) Photograph of the original environment. (Upper right) Raw 3D scan. (Bottom right) Plane feature segmentation and fitting. (Bottom**

left) final plane segmentation result. Image courtesy of J. Weingarten [331]. s2

### Imágenes extraídas de esta página

![Imagen extraída página 20](assets/page_20_img_01.png)

![Imagen extraída página 20](assets/page_20_img_02.png)

![Imagen extraída página 20](assets/page_20_img_03.png)

![Imagen extraída página 20](assets/page_20_img_04.png)

---

## Página 21

3. Identify a specific digital CMOS-based camera on the market. Using product specifications for this camera, collect and compute the following values. Show your derivations:
- Dynamic range

- Resolution (of a single pixel)

- Bandwidth

4. Stereo vision. Solve the system given by equations (4.60) and (4.61) and find the optimal
point that minimizes the distance between the optical rays passing through and . For doing this, observe that these two equations define two distinct lines in the 3D space. The problems consists in rewriting these two equations as the difference between 3D points along these two lines. Then, impose that the partial derivatives of this distance with respect to and equal zero. From this, you will obtain the two points along the two lines at minimum distance between each other. The optimal point can then be found as the middle point between those points.

5. Challenge Question.
Implement a basic two-view structure-from-motion algorithm from scratch: (a) Implement the basic Harris corner detector in Matlab. (b) Take two images of the same scene from different view points. (c) Extract and match Harris features using SSD. (d) Implement the 8-point algorithm to compute the essential matrix. (e) Compute rotation and translation up to a scale from the essential matrix. Disambiguate the four solutions using the cheirality constraint. x y z p× l p× r l r x y z
