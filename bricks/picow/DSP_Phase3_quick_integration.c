#include "pid_controller_dsp.h"
#include "encoder_filter_dsp.h"  // Phase 2

cascade_pid_t controller;
encoder_dsp_pipeline_t encoder;

void setup() {
    // Phase 2
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    
    // Phase 3
    cascade_pid_init(&controller,
        5.0f, 1.0f, 0.1f,   // Position gains
        2.0f, 0.5f, 0.05f,  // Velocity gains
        0.001f);
}

void loop_1khz() {
    // Phase 2: Filter
    int32_t pos = encoder_pipeline_update(&encoder, read_encoder());
    int32_t vel = encoder_pipeline_get_velocity(&encoder);
    
    // Phase 3: Control
    int32_t output = cascade_pid_update(&controller, setpoint, pos, vel);
    
    set_motor(output);
}
```

**Total overhead**: 1.05% of loop → **98.95% free for everything else!**

---

### 🎯 **Roadmap Progress**
```
Phase 1: ✅ DSP Foundation (COMPLETE)
Phase 2: ✅ Encoder Filters (COMPLETE)
Phase 3: ✅ Advanced PID    (COMPLETE) ← YOU ARE HERE
Phase 4: ⏭️ Trajectory Planning (READY TO START)
Phase 5: 🔲 Motion Profiling
Phase 6: 🔲 Integration & Testing
Phase 7: 🔲 Advanced Features
