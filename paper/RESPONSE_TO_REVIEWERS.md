# Response to Reviewer Comments

**Paper:** *Closed-Loop Large Language Model Task Planning for a Simulated Mobile
Manipulator: An Implementation Study and Honest Reliability Analysis* (ASET 2026)

We thank the reviewer for a thorough and constructive review. Every point has
been addressed in the revised manuscript. Below, each comment is placed
**side by side** with the change made and its location. Table numbers refer to
the compiled revision (tables auto-number in order of appearance).

---

## A. Identified weaknesses

| # | Reviewer comment | What we changed |
|---|---|---|
| W1 | Evaluation sample size is small (16 single-object, 4 two-object, 1 ambiguous). | Kept the true trial counts; added **Wilson 95% confidence intervals** to the results table and an explicit **"preliminary"** statistical note stating we draw only qualitative conclusions from the small-$N$ scenarios. (§VI Results, Table V + "Statistical note"; §VII Limitations.) |
| W2 | Baseline comparison is not fully balanced (tuned config, fewer trials). | We state explicitly that baseline and LLM rates use different denominators and tuned configurations, mark the baseline's excluded pre-fix runs with a footnote, and compare the two systems **qualitatively (determinism vs. adaptability)** rather than as a single win. (§VI-B, §VII, Fig. 4 caption.) |
| W3 | Broken figure reference "Fig. ??". | Verified every `\ref` now resolves to a defined `\label` (zero orphans); the "??" was a single-pass / stale-PDF artifact. Added an explicit note to **compile twice** and cross-referenced all four figures in text. (Header note; §III-C, §V, §VI.) |
| W4 | Architecture figure appears missing / incorrectly numbered. | The architecture figure is present as **Fig. 1** (native TikZ: NL command → LLM → tool server → Nav2 / MoveIt 2 / perception → Gazebo → feedback loop) and is now explicitly cited in §III-C. |
| W5 | LLM configuration not fully reported. | Added a dedicated **LLM configuration table** (Table II): exact model ID `claude-haiku-4-5-20251001`, temperature (provider default), `max_tokens`=1024, `max_turns` (25/30), 200 s tool timeout, tool-choice, retry policy (none — re-plan on failure), loop-breaker, prompt-caching state, and fixed system prompt. (§III-C.) |
| W6 | Simulation-only; grasp is a simplified detachable/kinematic attachment — state clearly. | Rewrote §VII to state plainly there is **no real-robot validation** and that the proximity-gated **kinematic attachment does not validate contact dynamics** (friction, force closure, slip). Also stated in the Abstract and §III-A. |
| W7 | Do not overclaim reliability. | Framed 81% as preliminary with CIs; reported the residual planner-loop and perception failures openly; degraded scenarios (S3 50%, S5) reported as limitations, not successes. (§VI, Table V, Table VII, §VII, Conclusion.) |
| W8 | Present the baseline comparison as flexibility-vs-determinism, not a performance win. | §VI-B and the Fig. 4 caption now frame S1-vs-S2 as **determinism vs. adaptability** on a shared action interface; a new control-systems subsection casts it as **feedforward vs. feedback** control. (§VI-B, §VI-A.) |

---

## B. Requested technical revisions

| # | Reviewer request | What we changed |
|---|---|---|
| R1 | Fix the broken "Fig. ??" reference (§III-C). | All cross-references resolve (verified programmatically); Fig. 1 is cited in §III-C; compile-twice note added. |
| R2 | Add/restore the system architecture figure (LLM, ROS 2 tool server, perception, Nav2, MoveIt 2, Gazebo, feedback loop). | **Fig. 1** contains exactly these blocks with the feedback loop highlighted; cited in §III-C. |
| R3 | Add a robot-tools table (name, input args, output, success condition, failure message). | **Table I** — all 11 tools with arguments, success condition, and representative failure message. (§III-B.) |
| R4 | Add an experimental-scenarios table (initial conditions, trials, success criteria, timeout/turn limit, failure criteria). | **Table III** — per-scenario initial conditions, $N$, success criterion, and turn/timeout failure limits. (§V.) |
| R5 | Increase the number of trials (baseline, two-object, ambiguous). | Not feasible for this revision (requires re-running the live simulation stack and additional API budget); addressed instead via R6. Stated transparently in §VII. |
| R6 | If more trials cannot be added, include confidence intervals or state results are preliminary. | Added **Wilson 95% CIs** (Table V) and an explicit preliminary-results statement. (§VI, §VII.) |
| R7 | Add a failure-mode table (planner loops, perception, reachability, localization, timeout). | **Table VII** — the five modes, their causes, and whether each is residual (frozen system) or fixed/mitigated during development. (§VI.) |
| R8 | Clarify whether YOLOv8 was used or only HSV. | Stated explicitly: the reported runs used the **default HSV colour-segmentation detector**; YOLOv8n is implemented but **was not used**. (§V "Perception configuration".) |
| R9 | Add main Nav2 / MoveIt 2 / Gazebo / perception / grasping parameters (planning timeout, velocity scaling, proximity threshold, physics, detection thresholds). | **Table IV** — DART physics, MPPI 20 Hz + 0.25 m/0.15 rad tolerance + 300 s nav timeout, RRTConnect, 0.8/0.4 arm scaling, 0.005 m grasp tolerance, HSV 50 px / YOLO 0.45, DetachableJoint 0.08 m proximity, 0.30 m nudge. (§V.) |
| R10 | Report computational cost (LLM calls, average response time, approximate API cost). | **Table VI** — 8.8 calls & ~1.8 s/call & ~\$0.047/task (S1); 23.5 calls & ~2.0 s/call & ~\$0.189/task (S3), computed from logs at Claude Haiku 4.5 list pricing. (§VI "Computational Cost".) |
| R11 | Strengthen the limitations: no real-robot validation; physical grasp/contact dynamics not validated. | §VII rewritten into three explicit subsections: no real-robot validation, kinematic-grasp / contact-dynamics caveat, small samples, and simplified environment. |
| R12 | Add more recent references (LLM robot planning, tool-calling for robotics, embodied agents, closed-loop execution). | Added PaLM-E, RT-2, VoxPoser, ReAct, Toolformer, and an LLM-for-robotics survey, all cited in Related Work. (§II, refs.) |

---

## C. Additional change

| Request | What we changed |
|---|---|
| Add Dr. Ammar Natsheh to the author list, before F. Tesfaye. | Author list updated: **A. Natsheh, F. Tesfaye, M. Teshome, Y. Benachour.** A new control-systems subsection (§VI-A, "A Control-Systems Perspective") provides the applied-engineering interpretation of the closed-loop behavior (hybrid supervisory control, feedback vs. feedforward, planner loops as limit cycles, impulsive grasp coupling, kinematic-conditioning margin) for his area of expertise. |

---

*All changes are reflected in `aset2026_paper.tex`. A companion technical dossier
(`PAPER_DOSSIER.md`) documents every version, coordinate, parameter, and result
with provenance.*
