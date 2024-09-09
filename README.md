# 基于改进粒子群算法的多无人机协同航迹规划——Uav-track-collaborative-planning-based-on-pso-algorithm

---
使用matlab实现，PSO_code文件夹里分三个文件夹<br/>
- PSO multi-population single-track code：为带地形和威胁区，地图大小为[500m，500m，100m]的单架无人机航迹规划
- PSO time coordination code：为带地形和威胁区，地图大小为[500m，500m，100m]的五架无人机的协同航迹规划
- PSO collaboration breakout scenario code：为带威胁区，地图大小为[200km，200km，1000m]的五架无人机的协同航迹规划

Implementation using matlab，PSO_code folder divided into three folders <br/>
- PSO multi-population single-track code: It is a single UAV track planning with terrain and threat area and map size [500m，500m，100m]
- PSO time coordination code: Coordinated flight path planning for five UAVs with terrain and threat area and map size [500m，500m，100m]
- PSO collaboration breakout scenario code: Indicates the cooperative flight path planning of five UAVs with the threat area and a map size of [200km，200km，1000m]
---


&emsp;&emsp;无人机集群协同航迹规划通过规划集群的最优航迹，协调多种类型的多架无人机，在同一时刻到达目标，提高无人机的整体突防能力，实现饱和攻击。粒子群算法在适应性、可扩展性、全局寻优和并行处理等方面展现出了卓越的性能，因而被广泛应用于路径规划研究领域。本项目采用改进粒子群算法对无人机集群协同航迹进行规划，最后通过仿真实验验证算法的可行性和有效性。 通过动态粒子群算法对单架无人机三个威胁区进行了单轨迹仿真和五架无人机三个威胁区的协同航迹规划进行了仿真。结果表明，可以规划出无人机航迹并且成功避开威胁区，协同航迹能够同时到达目标区域对敌进行打击。<br/>

&emsp;&emsp;By planning the optimal flight path of the cluster, multiple UAVs of various types can be coordinated to reach the target at the same time, so as to improve the overall penetration capability of UAVs and realize saturation attack. Particle swarm optimization (PSO) is widely used in path planning research because of its excellent performance in adaptability, scalability, global optimization and parallel processing. In this project, the improved particle swarm optimization algorithm is used to plan the cooperative flight path of UAV cluster. Finally, the feasibility and effectiveness of the algorithm are verified by simulation experiments. Dynamic particle swarm optimization (PSO) was used to simulate the three threat zones of a single UAV and the cooperative track planning of three threat zones of five UAVs. The results show that the UAV's flight path can be planned and successfully avoided the threat area, and the cooperative flight path can simultaneously reach the target area to strike the enemy. <br/>
