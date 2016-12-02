class Normalizer:
    def __init__(self,
                 real_val_max, real_val_min,
                 norm_val_max, norm_val_min,
                 apply_clamp=True):
        self.real_val_max = real_val_max
        self.real_val_min = real_val_min
        self.real_val_diff = real_val_max - real_val_min
        self.norm_val_max = norm_val_max
        self.norm_val_min = norm_val_min
        self.norm_val_diff = norm_val_max - norm_val_min
        self.apply_clamp = apply_clamp
        #
        # Check if wrong values exist in the setting
        # e.g. min <= max or abs(max-min) is too small
        #
        for v in self.real_val_diff:
            if v <= 0.0 or abs(v) < 1.0e-08:
                raise Exception('Normalizer', 'wrong values')
        for v in self.norm_val_diff:
            if v <= 0.0 or abs(v) < 1.0e-08:
                raise Exception('Normalizer', 'wrong values')

    def real_to_norm(self, val):
        val_0_1 = (val - self.real_val_min) / self.real_val_diff
        if self.apply_clamp:
            self._clamp(val_0_1)
        return self.norm_val_min + self.norm_val_diff * val_0_1

    def norm_to_real(self, val):
        val_0_1 = (val - self.norm_val_min) / self.norm_val_diff
        if self.apply_clamp:
            self._clamp(val_0_1)
        return self.real_val_min + self.real_val_diff * val_0_1

    def _clamp(self, val):
        def clamp(_val, _min, _max):
            if _val < _min:
                return _min
            elif _val > _max:
                return _max
            return _val

        for i in range(len(val)):
            val[i] = clamp(val[i], 0.0, 1.0)
