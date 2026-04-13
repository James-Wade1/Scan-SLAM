### **Tests and Results**

General Results
- Chart showing 1) true position, 2) odometry, and 3) corrected trajectory
- Uncertainty of current position over time
- Absolute trajectory error
- Relative pose error
- Runtime for front end vs backend

Front End Results
- Branch and Bound values

Back End Results
- Graph error over time
- Plot graph error before and after (perhaps a bar chart where x axis is each time the graph optimization was done?)
- How many iterations for the graph optimization?
- Optimization time with sparse vs non sparse solver

Stretch Results
- Occupancy grid?

| Test Name | Description|
| --- | --- |
| Test 1 | Ros bag with all topics. The odometry drifted but the trajectory corrected itself. No search window was used for scan matching |
| Test 2 | Ros bag with all topics. This one was made after the changes to the posterior and how we were scoring |
| Test 3 | Ros bag with all topics. This one was made after adding occupancy grid support |
| Test 4 | Ros bag with all topics. Full exploration. |
| Test 5 | Ros bag with all topics. Full exploration with new stamped topics. |
| Test 6 | Ros bag with all topics. Full exploration without optimization. |