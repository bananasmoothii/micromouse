## Resolution `decel_start_distance` in case there isn't enough time for going full speed

### Finding the Transition Position $x_1$ using Torricelli's equation

**1. Acceleration phase (from $x_0$ to $x_1$)**
The object starts at $x_0$ with velocity $v_0$ and accelerates at $+a$ until it reaches $v_{max}$ at position $x_1$.

$$v_{max}^2 - v_0^2 = 2a(x_1 - x_0)$$

**2. Deceleration phase (from $x_1$ to $d$)**
The object brakes from $v_{max}$ at a rate of $-a$ until it reaches its final velocity $v_f$ at the final position $d$.

$$v_f^2 - v_{max}^2 = 2(-a)(d - x_1)$$

**3. Eliminating $v_{max}$ to find $x_1$**
From the first equation, we can express $v_{max}^2$:

$$v_{max}^2 = v_0^2 + 2a(x_1 - x_0)$$

Now, substitute this into the second equation to eliminate the unknown $v_{max}$:

$$v_f^2 - \left[ v_0^2 + 2a(x_1 - x_0) \right] = -2a(d - x_1)$$

Expand both sides:

$$v_f^2 - v_0^2 - 2ax_1 + 2ax_0 = -2ad + 2ax_1$$

Move all the $x_1$ terms to the right side, and everything else to the left:

$$v_f^2 - v_0^2 + 2ax_0 + 2ad = 4ax_1$$

Factor out the $2a$ on the left side:

$$v_f^2 - v_0^2 + 2a(x_0 + d) = 4ax_1$$

Finally, divide by $4a$ to isolate $x_1$:

$$x_1 = \frac{v_f^2 - v_0^2 + 2a(x_0 + d)}{4a}$$

---

**💡 Physics Sanity Check:**
You can rewrite this final result by splitting the fraction:

$$x_1 = \frac{x_0 + d}{2} + \frac{v_f^2 - v_0^2}{4a}$$

This form is beautiful because it makes total physical sense:

* The term **$\frac{x_0 + d}{2}$** is the exact midpoint of the total distance.
* If the initial and final velocities are the same ($v_0 = v_f$), the second fraction becomes zero, and $x_1$ falls
  perfectly exactly in the middle. The acceleration and braking phases are perfectly symmetrical!
* If you need to end up with a higher velocity than you started with ($v_f > v_0$), the second fraction is positive,
  meaning you spend more distance accelerating and $x_1$ shifts further down the track.