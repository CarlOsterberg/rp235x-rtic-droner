

pub struct MotorOutputs {
    fl: f32,
    fr: f32,
    bl: f32,
    br: f32,
}

impl MotorOutputs {
    pub fn get_fl(&self) -> f32
    {
        self.fl
    }

    pub fn get_bl(&self) -> f32
    {
        self.bl
    }

    pub fn get_br(&self) -> f32
    {
        self.br
    }

    pub fn get_fr(&self) -> f32
    {
        self.fr
    }
}

pub fn mix(base_throttle: f32, roll_out: f32, pitch_out: f32, yaw_out: f32) -> MotorOutputs {
    MotorOutputs {
        fr: base_throttle + roll_out + pitch_out + yaw_out,
        bl: base_throttle - roll_out - pitch_out + yaw_out,
        fl: base_throttle - roll_out + pitch_out - yaw_out,
        br: base_throttle + roll_out - pitch_out - yaw_out,
    }
}

pub fn mix_and_clamp(
    base_throttle: f32,
    roll_out: f32,
    pitch_out: f32,
    yaw_out: f32,
    min: f32,
    max: f32,
) -> MotorOutputs {
    let mut m = mix(base_throttle, roll_out, pitch_out, yaw_out);

    // find the largest overshoot beyond [min, max] across all four motors
    let highest = m.fl.max(m.fr).max(m.bl).max(m.br);
    let lowest  = m.fl.min(m.fr).min(m.bl).min(m.br);

    if highest > max {
        let excess = highest - max;
        m.fl -= excess; m.fr -= excess; m.bl -= excess; m.br -= excess;
    }
    if lowest < min {
        let deficit = min - lowest;
        m.fl += deficit; m.fr += deficit; m.bl += deficit; m.br += deficit;
    }

    // final safety clamp in case both conditions triggered and pushed past the other bound
    m.fl = m.fl.clamp(min, max);
    m.fr = m.fr.clamp(min, max);
    m.bl = m.bl.clamp(min, max);
    m.br = m.br.clamp(min, max);

    m
}