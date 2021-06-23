# Determination of uncertainty

In the camera pose tracking process of ORB SLAM2, [g2o](https://github.com/RainerKuemmerle/g2o) is used to solve the graph optimization problem. In the [g2o paper](https://www.researchgate.net/publication/224252449_G2o_A_general_framework_for_graph_optimization), the linear system for the optimization is introduced in equation (24)
$$
\begin{pmatrix}
	H_{\rm pp}     & H_{\rm pl} \\
	H_{\rm pl}^{T} & H_{\rm ll}
\end{pmatrix}
\begin{pmatrix}
	\Delta {\bf x}_{\rm p}^{*} \\
	\Delta {\bf x}_{\rm l}^{*}
\end{pmatrix}
=
\begin{pmatrix}
	-{\bf b}_{\rm p} \\
	-{\bf b}_{\rm l} \\
\end{pmatrix},
$$
where subscripts ${\rm p}$ and ${\rm l}$ represent pose and landmark, $H$ is Hessian, $\Delta {\bf x}^{*}$ is the increment, and ${\bf b}$ is the gradient. Hessian regarding the pose increment ${\bf x}_{\rm p}^{*}$ is formed by taking the Schur complement as
$$
(H_{\rm pp} - H_{\rm pl} H_{\rm ll} H_{\rm pl}^{-1}) \Delta {\bf x}_{\rm p}^{*} = -{\bf b}_{\rm p} - H_{\rm pl} H_{\rm ll}^{-1} {\bf b}_{\rm l}.
$$
This is equation (25) of the g2o paper. Here, we define $H$ as
$$
H \overset{\text{def}}{=} H_{\rm pp} - H_{\rm pl} H_{\rm ll} H_{\rm pl}^{-1}.
$$
Then, we determine the uncertainty, i.e., covariance matrix, of the estimate by ORB SLAM2 as
$$
R = \frac{1}{\sigma^{2}} H^{-1},
$$
where $\sigma^{2}$ is the scale factor and is must be positive.