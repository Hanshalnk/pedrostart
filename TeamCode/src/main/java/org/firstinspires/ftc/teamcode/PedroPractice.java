package org.firstinspires.ftc.teamcode;

// ---- FTC SDK ----
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// ---- Pedro Pathing (v2) ----
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

// ---- Your Quickstart constants ----
import org.firstinspires.ftc.teamcode.constants.Constants;

// ---- FTC Dashboard (optional; requires dependency) ----
// TeamCode/build.gradle needs: implementation "com.acmerobotics.dashboard:dashboard:0.4.13"
// If you do NOT add that dependency, comment out these four imports AND the code that uses them.
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

@Autonomous(name = "Pedro Practice (Panels + Dashboard Tuning)", group = "Practice")
public class PedroPractice extends OpMode {

    // --- Pedro core objects ---
    private Follower follower;   // Drives paths, tracks pose
    private Timer pathTimer;     // Simple timer for state timing

    // --- Simple state machine to run two paths ---
    private int pathState = 0;   // 0 -> first path, 1 -> second, 2 -> done

    // --- Dashboard handle (optional) ---
    private FtcDashboard dash;

    // --------------------------------------------------------------------------------------------
    // NOTES: Coordinates are inches; heading in radians. Adjust to your field orientation.
    // --------------------------------------------------------------------------------------------
    private final Pose startPose        = new Pose(12, 12, Math.toRadians(90));
    private final Pose curveControlPose = new Pose(24, 36, Math.toRadians(90));
    private final Pose curveEndPose     = new Pose(36, 12, Math.toRadians(90));
    private final Pose finalPose        = new Pose(36, 36, Math.toRadians(90));

    // Paths we’ll build once in init()
    private Path curvePath;
    private Path finalForward;

    private void buildPaths() {
        // Curved segment: start -> control -> end
        curvePath = new Path(new BezierCurve(
                new Point(startPose),
                new Point(curveControlPose),
                new Point(curveEndPose)
        ));
        // Keep heading constant here (or change it if you want to turn during the curve)
        curvePath.setLinearHeadingInterpolation(
                startPose.getHeading(), curveEndPose.getHeading()
        );

        // Straight segment: curve end -> final
        finalForward = new Path(new BezierLine(
                new Point(curveEndPose),
                new Point(finalPose)
        ));
        finalForward.setLinearHeadingInterpolation(
                curveEndPose.getHeading(), finalPose.getHeading()
        );
    }

    // Simple autonomous state machine
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(curvePath);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(finalForward);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(-1); // done
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    // ===== FTC lifecycle =====

    @Override
    public void init() {
        // (Optional) Dashboard mirror
        // If you didn’t add the dashboard dependency, comment out these two lines:
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        pathTimer = new Timer();

        // Build follower from constants (drivetrain + localizer + constraints)
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose before using any paths
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addLine("Initialized (Pedro v2 + Panels + Dashboard)");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();      // REQUIRED every cycle
        autonomousPathUpdate(); // run the state machine

        // --- Dashboard packet (optional) ---
        // If you removed the dashboard dependency, comment this block out.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("state", pathState);
        packet.put("x", follower.getPose().getX());
        packet.put("y", follower.getPose().getY());
        packet.put("heading_deg", Math.toDegrees(follower.getPose().getHeading()));

        Canvas field = packet.fieldOverlay();
        field.setStrokeWidth(2);
        // Robot pose marker + heading ray
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = follower.getPose().getHeading();
        field.strokeCircle(x, y, 2);
        field.strokeLine(x, y, x + 4*Math.cos(h), y + 4*Math.sin(h));
        // Waypoints
        field.strokeCircle(startPose.getX(),        startPose.getY(),        1.5);
        field.strokeCircle(curveControlPose.getX(), curveControlPose.getY(), 1.5);
        field.strokeCircle(curveEndPose.getX(),     curveEndPose.getY(),     1.5);
        field.strokeCircle(finalPose.getX(),        finalPose.getY(),        1.5);
        dash.sendTelemetryPacket(packet);
        // ------------------------------------

        // Mirror to Driver Station
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading(rad)", h);
        telemetry.update();
    }
}
