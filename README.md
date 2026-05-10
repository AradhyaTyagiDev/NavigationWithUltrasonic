# NavigationWithUltrasonic
Navigation With 1 Ultrasonic Sensor

# Filter used:
1. Validity Check: Range Validation
	1. Timeout handling: When timeout occurs:Treat as: "No valid obstacle detected"
2. Confidence-Based Filtering: Each reading gets a **confidence score (0–1)**
	1. Normal change: 1.0
	2. Small jump (<10 cm): 0.8
	3. Large jump (>50 cm): 0.2
	4. Out of range: 0
3. Outlier Rejection (Safety layer)
	1. Ignore: 0, 400, Sudden jumps, Previous = 20 cm, New = 150 cm ❌ (ignore)
4. DEAD ZONE FILTER: Prevents tiny sensor jitter. Without dead zone: Robot micro-adjusts constantly. Dead zone stabilizes behavior.
	1. Dead Zone: Ignore small fluctuations: if (abs(new - old) < 2 cm → ignore)
5. Average < Exponential Moving Average (EMA) : —**EMA reacts faster + smoother
	1. filtered = alpha × new + (1-alpha) × old
6. Adaptive EMA: alpha changes dynamically
	1. velocity-based adaptive EMA: 
		1. slow change → low alpha
		2. fast change → high alpha
	2. If implement Adoptive EMA, then no need for Fixed EMA
7. Velocity Awareness: Calculate the velocity and boundary check
8. Stability Check (for decisions)
	1. Stability Window (Decision Layer) (3 frames): 👉 Don’t react instantly
