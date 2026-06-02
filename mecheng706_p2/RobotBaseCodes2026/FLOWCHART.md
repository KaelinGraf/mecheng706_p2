# Tracking flow chart

This diagram reflects the SEARCH sub-behaviour transitions in [tracking.cpp](tracking.cpp).

```mermaid
flowchart TD
    A[Start in FIND_FIRE] --> B{Wall / obstacle ahead?}
    B -- No --> A
    B -- Yes --> C[AVOID]

    C --> D{Obstacle clear and AVOID_STRAFE_MS passed?}
    D -- Yes --> E[RETURN_TO_HEADING]
    D -- No --> F{AVOID timeout?}
    F -- Yes --> G[Rotate in place]
    F -- No --> H{Obstacle side / left / right?}

    H -- Left --> I[Strafe / rotate around left obstacle]
    H -- Right --> J[Strafe / rotate around right obstacle]
    H -- Side --> K[Move around side obstacle]
    H -- None --> L{Obstacle ahead and aimed?}

    L -- Yes --> M{Turret at fire?}
    M -- Yes --> N[MOVE_TO_FIRE]
    M -- No --> O[Fire is behind obstacle; keep avoiding]
    L -- No --> P[Turn away from obstacle]

    E --> Q{Heading error > 0.12 rad?}
    Q -- Yes --> R[RETURN_TO_HEADING continues until heading is restored]
    Q -- No --> S{Resume to move?}
    S -- Yes --> N
    S -- No --> A

    Q -. obstacle check runs first .-> B
    R -. obstacle detected mid-recovery .-> C

    N --> T{Fire close, aimed, and turret locked?}
    T -- Yes --> U[EXTINGUISH]
    T -- No --> V[Adjust heading toward fire]

    N --> W{Turret lock lost?}
    W -- Yes --> A
    W -- No --> N

    style C fill:#ffe1b3,stroke:#b36b00,color:#000
    style E fill:#dbeafe,stroke:#2563eb,color:#000
    style N fill:#dcfce7,stroke:#15803d,color:#000
    style U fill:#fee2e2,stroke:#b91c1c,color:#000
```

State mapping used by the code:

- FIND_FIRE: search / wall-follow mode
- AVOID: obstacle response
- RETURN_TO_HEADING: restore the original bearing after avoidance
- MOVE_TO_FIRE: steer toward the detected fire
- EXTINGUISH: fan-on state once the robot is close enough

Important detail: the obstacle / wall check has priority over RETURN_TO_HEADING in the code, so recovery can be interrupted and sent back into AVOID before it reaches the heading-restored branch.

If you want, I can also turn this into a compact ASCII version for the serial log or add a version annotated with the exact sensor conditions from the code.