<h2>Detailed Breakdown</h2>
<h3>1. Initialization:</h3>

Each system component is initialized and logs confirmation.
Path is planned using the A* algorithm.
<h3>2.Step-by-Step Simulation:</h3>

<h4>For each step:</h4>
<p>Sensors: Camera, LIDAR, and radar process environmental data.</p><br>
<p>Decision-Making: Detects objects (e.g., pedestrian, car, stop sign) and decides the action (e.g., "STOP", "SLOW_DOWN", or "DRIVE_NORMAL").</p><br>
<p>Control: Adjusts speed and steering based on the decision.</p><br>
<p>Communication: Sends traffic updates for V2V.</p><br>
<h3>Special Cases:</h3>

<p>Emergency stops at step 0 (pedestrian detected), step 6 (stop sign), and step 8 (red traffic light).</p><br>
<p>Speed adjustments for slower navigation at step 4 (bicycle detected).</p><br>
<h3>Completion:</h3>
<p>The car successfully completes its journey, logging each step.</p>
