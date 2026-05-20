# Student Tracking Concept

From the student's point of view, this project is meant to be a radar tracking assignment where the simulator provides a realistic world, but not the tracking solution.

Conceptually, the student experience should be:

- The world and sensor physics already exist.
  - The car motion, propagation, echoes, and radar measurement process are handled by the simulator in `lib/`.
  - The student does not need to build a radar simulator from scratch.

- The student is given the radar's known physical parameters.
  - They know things like carrier frequency `f_c`, IQ bandwidth, sample rate, and probe placement inputs.
  - These are the constraints of the sensing system.

- Everything after that is the student's problem to solve.
  - They decide how to form the chirp.
  - They decide chirp duration and frequency law.
  - They decide how to use the probe.
  - They decide how to process the received signal.
  - They design and implement the tracker.

- Their job is not to simulate truth, but to infer truth from measurements.
  - The simulator produces what the radar "sees."
  - The student tracker must estimate target state from those measurements.

- The framework should separate estimate from truth.
  - `step()` should produce the student's tracker estimate.
  - Truth should be available separately for evaluation, debugging, and grading.
  - This keeps the assignment honest: they are not handed the answer as the main output.

- The public API should be intentionally small.
  - Students should interact with a very simple harness like `init()` and `step()`.
  - The complexity should live in the tracking logic, not in learning a huge framework API.

So the educational objective is:

- give students a realistic radar sensing pipeline,
- hide the low-level simulator implementation,
- force them to own the signal-processing and tracking decisions,
- and let them compare their estimated state against ground truth afterward.

In one sentence:

- You want students to build a tracker from radar returns, not build a simulator, and then evaluate how well their tracker reconstructs the true car state.
