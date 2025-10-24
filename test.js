
            // Assumes: rclnodejs Node is available as this.environment.variables.ros_node
            // Publishes: geometry_msgs/msg/TwistStamped on /cmd_vel
            // Subscribes: nav_msgs/msg/Odometry on /odom

            const node = this.environment.variables.ros_node;

            // ====================== USER TUNABLES ======================
            const hz = 200;                                            // control loop frequency [Hz]
            const duration_s = 5;                                // hard stop safety duration
            const targetYawRad = Math.PI + Math.PI % (2*Math.PI);                        // desired yaw (radians). Update at runtime if you like.
            const targetLinear = 0.0;                        // desired forward speed [m/s] (set >0 to drive while holding heading)

            // PID gains for yaw control (start modest; tune on your robot)
            const Kp = 1.8;
            const Ki = 0.1;
            const Kd = 0.0;

            // Output/slew limits
            const maxAngVel = 0.7;                             // [rad/s]
            const maxAngAcc = 1.0;                             // [rad/s^2] change allowed
            const maxLinVel = targetLinear;                             // [m/s]
            const maxLinAcc = 0.7;                             // [m/s^2]

            const yawDeadband = 0.01;
            const finishYawError = 0.02;
            const finishHoldTime_s = 0.3;

            const odomTimeout_s = 0.25;
            const imuTimeout_s    = 0.25;

            const fusionHz = hz;
            const tau_s = 5.0;
            const alpha = Math.exp(-1.0 / (tau_s * fusionHz));
            const useCovarianceWeight = true;
            const imuMaxVarForFullTrust = 0.05;

            const imuDeclination_rad = 0.0;

            // Watchdog (IMU only)
            //const imuTimeout_s    = 0.25;                         // stop if IMU older than this

            // If your IMU yaw needs a fixed offset (e.g., declination), set here:
            const imuYawOffset_rad = 0.0;

            const frame_id = "base_link";
            // ===========================================================
            
            // ===== Helpers =====
            function nowStamp() {
                const ms = Date.now();
                return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
            }
            function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
            function wrapPi(a) { const t = ((a + Math.PI) % (2*Math.PI) + 2*Math.PI) % (2*Math.PI); return t - Math.PI; }
            function norm(q){ const n = Math.hypot(q.x, q.y, q.z, q.w) || 1.0; return {x:q.x/n, y:q.y/n, z:q.z/n, w:q.w/n}; }
            function quatToEulerRPY(qraw){
                const q = norm(qraw); const x=q.x,y=q.y,z=q.z,w=q.w;
                const sinr_cosp = 2*(w*x + y*z), cosr_cosp = 1 - 2*(x*x + y*y);
                const roll = Math.atan2(sinr_cosp, cosr_cosp);
                const sinp = 2*(w*y - z*x);
                const pitch = (Math.abs(sinp) >= 1) ? Math.sign(sinp)*(Math.PI/2) : Math.asin(sinp);
                const siny_cosp = 2*(w*z + x*y), cosy_cosp = 1 - 2*(y*y + z*z);
                const yaw = Math.atan2(siny_cosp, cosy_cosp);
                return {roll: roll, pitch: pitch, yaw: yaw};
            }

            // ===== State (IMU only) =====
            let lastIMU    = null;         // {t_ms, yaw, var}
            let integ = 0, prevErr = 0;
            let lastCmd = { lin: 0, ang: 0 };
            let finishStartMs = null;

            // ===== Subscriptions (your IMU topic) =====
            const imuTopics = [
                {type: "sensor_msgs/msg/Imu", name: "/a300_00041/sensors/imu_0/data"}
            ];

            function onImu(msg, topicName){
                const cov0 = (msg.orientation_covariance && msg.orientation_covariance.length > 0) ? msg.orientation_covariance[0] : 0;
                if (cov0 === -1) {
                    this.environment.services.console.warn("[IMU] " + topicName + ": invalid orientation (cov[0] = -1).");
                    return;
                }
                const rpy = quatToEulerRPY(msg.orientation);
                let yaw = wrapPi(rpy.yaw + imuYawOffset_rad);
                const yawVar = (msg.orientation_covariance && msg.orientation_covariance.length >= 9) ? msg.orientation_covariance[8] : cov0;

                lastIMU = { t_ms: Date.now(), yaw: yaw, var: (typeof yawVar === "number" ? Math.max(0, yawVar) : 0.0) };

                this.environment.variables.imu = msg;
                this.environment.variables.imu_quat = msg.orientation;
                this.environment.variables.imu_orientation_cov = msg.orientation_covariance;
                this.environment.variables.roll_pitch_yaw = rpy;
                this.environment.variables.yaw_rad = yaw;
                this.environment.variables.yaw_deg = yaw * 180/Math.PI;
                this.environment.variables.imu_source_topic = topicName;
            }

            const imuSubs = imuTopics.map(function(spec){
                return node.createSubscription(spec.type, spec.name, function(msg){ onImu.call(this,msg,spec.name); }.bind(this));
            }.bind(this));

            this.environment.services.console.log(
                "Listening for sensor_msgs/msg/Imu on topics: " + imuTopics.map(function(t){ return t.name; }).join(", ") + "."
            );

            // ===== Publisher =====
            const pub = node.createPublisher("geometry_msgs/msg/TwistStamped", "cmd_vel");

            // ===== Timing =====
            const period_s = 1.0 / hz;
            const period_ns_bigint = BigInt(Math.round(1000000000 / hz));
            const duration_ns_bigint = BigInt(Math.round(duration_s * 1000000000));

            // ===== Control using IMU yaw only =====
            function imuAvailable() { return lastIMU && (Date.now() - lastIMU.t_ms) <= imuTimeout_s*1000; }

            function publishCmd(lin, ang) {
                pub.publish({
                    header: { stamp: nowStamp(), frame_id: frame_id },
                    twist: { linear: {x: lin, y:0, z:0}, angular: {x:0, y:0, z: ang} }
                });
                lastCmd = { lin: lin, ang: ang };
            }

            function controlTick() {
                const t_ms = Date.now();

                if (!imuAvailable()) {
                    integ = 0; prevErr = 0; finishStartMs = null;
                    if (Math.abs(lastCmd.lin) > 1e-6 || Math.abs(lastCmd.ang) > 1e-6) publishCmd(0,0);
                    return;
                }

                const yaw = lastIMU.yaw;

                let err = wrapPi(targetYawRad - yaw);
                if (Math.abs(err) < yawDeadband) err = 0;

                integ += err * period_s;
                const deriv = (err - prevErr) / period_s;
                let u = Kp*err + Ki*integ + Kd*deriv;

                let angCmd = clamp(u, -maxAngVel, +maxAngVel);
                if (u !== angCmd) integ *= 0.9;

                const maxDeltaAng = maxAngAcc * period_s;
                angCmd = lastCmd.ang + clamp(angCmd - lastCmd.ang, -maxDeltaAng, +maxDeltaAng);

                let linTarget = clamp(targetLinear, -maxLinVel, +maxLinVel);
                const maxDeltaLin = maxLinAcc * period_s;
                let linCmd = lastCmd.lin + clamp(linTarget - lastCmd.lin, -maxDeltaLin, +maxDeltaLin);

                const withinFinish = Math.abs(err) <= finishYawError && Math.abs(linTarget) < 1e-6;
                if (withinFinish) {
                    if (finishStartMs == null) finishStartMs = t_ms;
                    if ((t_ms - finishStartMs)/1000.0 >= finishHoldTime_s) {
                        publishCmd(0,0);
                        prevErr = err;
                        return;
                    }
                } else {
                    finishStartMs = null;
                }

                publishCmd(linCmd, angCmd);
                prevErr = err;
            }

            // ===== Timers =====
            const ctlTimer = node.createTimer(period_ns_bigint, controlTick);
            const stopTimer = node.createTimer(duration_ns_bigint, function() {
                ctlTimer.cancel();
                publishCmd(0,0);
                stopTimer.cancel();
                if (typeof next === "function") next();
            });

            // For visibility
            this.environment.variables.nextTimerDuration = "PT" + duration_s + "S";
            this.environment.services.console.log(
                "IMU-only heading control @" + hz + "Hz. targetYaw=" + targetYawRad.toFixed(3) + " rad, v=" + targetLinear + " m/s for " + duration_s + " second(s)."
            );
            