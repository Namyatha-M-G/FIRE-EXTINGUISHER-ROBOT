/* =========================================================
   ROBOT STATES
========================================================= */

const states = [

    "IDLE",

    "SCAN",

    "NAVIGATING",

    "FORWARD",

    "TURN_LEFT",

    "TURN_RIGHT",

    "EXTINGUISH",

    "VERIFY",

    "ARRIVED",

    "SEARCHING",

    "RETURNING_HOME"

];



/* =========================================================
   ROBOT IP
========================================================= */

let serverIp = "192.168.4.1";



/* =========================================================
   CONNECT ROBOT
========================================================= */

function connectRobot() {

    serverIp =
        document
            .getElementById("robot-ip")
            .value
            .trim();


    if (serverIp === "") {

        alert(
            "Please enter the robot IP address."
        );

        return;
    }


    fetchStatus();
}



/* =========================================================
   FETCH ROBOT STATUS
========================================================= */

function fetchStatus() {

    fetch(
        `http://${serverIp}/status`
    )

        .then(response => {

            if (!response.ok) {

                throw new Error(
                    "Robot did not respond."
                );
            }


            return response.json();

        })


        .then(data => {

            console.log(
                "Robot data:",
                data
            );


            /* -----------------------------
               CONNECTION
            ----------------------------- */

            document
                .getElementById(
                    "connection-status"
                )
                .textContent =
                "● ONLINE";


            document
                .getElementById(
                    "connection-status"
                )
                .className =
                "online";


            document
                .getElementById(
                    "robot-connection"
                )
                .textContent =
                "ONLINE";


            document
                .getElementById(
                    "robot-connection"
                )
                .className =
                "value online";



            /* -----------------------------
               UPDATE DASHBOARD
            ----------------------------- */

            updateDashboard(data);



            /* -----------------------------
               FETCH AGAIN AFTER 1 SECOND
            ----------------------------- */

            setTimeout(
                fetchStatus,
                1000
            );

        })


        .catch(error => {

            console.log(
                "Connection error:",
                error
            );


            /* -----------------------------
               SHOW OFFLINE
            ----------------------------- */

            document
                .getElementById(
                    "connection-status"
                )
                .textContent =
                "● OFFLINE";


            document
                .getElementById(
                    "connection-status"
                )
                .className =
                "offline";


            document
                .getElementById(
                    "robot-connection"
                )
                .textContent =
                "OFFLINE";


            document
                .getElementById(
                    "robot-connection"
                )
                .className =
                "value offline";


            document
                .getElementById(
                    "last-update"
                )
                .textContent =
                "Connection failed.";

        });

}



/* =========================================================
   UPDATE DASHBOARD
========================================================= */

function updateDashboard(data) {


    /* =====================================================
       ROBOT MODE
    ====================================================== */

    document
        .getElementById(
            "robot-mode"
        )
        .textContent =

        data.auto_mode
            ? "AUTONOMOUS"
            : "MANUAL";



    /* =====================================================
       ROBOT STATE
    ====================================================== */

    document
        .getElementById(
            "robot-state"
        )
        .textContent =

        states[data.state]
            || "UNKNOWN";



    /* =====================================================
       PUMP
    ====================================================== */

    document
        .getElementById(
            "pump"
        )
        .textContent =

        data.pump
            ? "ON"
            : "OFF";


    document
        .getElementById(
            "robot-pump"
        )
        .textContent =

        data.pump
            ? "ON"
            : "OFF";



    /* =====================================================
       ROBOT POSITION
    ====================================================== */

    document
        .getElementById(
            "robot-x"
        )
        .textContent =

        Number(data.x)
            .toFixed(2)
            + " m";


    document
        .getElementById(
            "robot-y"
        )
        .textContent =

        Number(data.y)
            .toFixed(2)
            + " m";


    document
        .getElementById(
            "heading"
        )
        .textContent =

        (
            Number(data.heading)
            * 180
            / Math.PI
        )
            .toFixed(1)
            + "°";



    /* =====================================================
       ROBOT FLAME SENSORS
    ====================================================== */

    setFlameValue(
        "flame-left",
        data.flame_l
    );


    setFlameValue(
        "flame-front",
        data.flame_f
    );


    setFlameValue(
        "flame-right",
        data.flame_r
    );



    /* =====================================================
       MOTOR SPEED
    ====================================================== */

    if (
        data.motor_speed !==
        undefined
    ) {

        document
            .getElementById(
                "motor-speed"
            )
            .textContent =
            data.motor_speed
            + "%";
    }



    /* =====================================================
       SENSOR NODE 1
    ====================================================== */

    updateNode(

        1,

        data.node1_name,

        data.node1_flame,

        data.node1_gas,

        data.node1_alert,

        data.node1_x,

        data.node1_y

    );



    /* =====================================================
       SENSOR NODE 2
    ====================================================== */

    updateNode(

        2,

        data.node2_name,

        data.node2_flame,

        data.node2_gas,

        data.node2_alert,

        data.node2_x,

        data.node2_y

    );



    /* =====================================================
       ALERT COUNT
    ====================================================== */

    let alertCount = 0;


    if (data.node1_alert) {

        alertCount++;

    }


    if (data.node2_alert) {

        alertCount++;

    }


    document
        .getElementById(
            "alert-count"
        )
        .textContent =
        alertCount;



    /* =====================================================
       ALERT SUMMARY
    ====================================================== */

    updateAlertSummary(

        data,

        alertCount

    );



    /* =====================================================
       LAST UPDATE
    ====================================================== */

    document
        .getElementById(
            "last-update"
        )
        .textContent =

        "Last update: "
        + new Date()
            .toLocaleTimeString();

}



/* =========================================================
   FLAME SENSOR DISPLAY
========================================================= */

function setFlameValue(

    elementId,

    detected

) {

    const element =
        document
            .getElementById(
                elementId
            );


    if (detected) {

        element.textContent =
            "FIRE";

        element.className =
            "value fire";

    }

    else {

        element.textContent =
            "OK";

        element.className =
            "value normal";

    }

}



/* =========================================================
   UPDATE SENSOR NODE
========================================================= */

function updateNode(

    id,

    name,

    flame,

    gas,

    alert,

    x,

    y

) {


    /* =====================================================
       NODE NAME
    ====================================================== */

    document
        .getElementById(
            `node${id}-name`
        )
        .textContent =

        name
            || `Node ${id}`;



    /* =====================================================
       FLAME
    ====================================================== */

    const flameElement =
        document
            .getElementById(
                `node${id}-flame`
            );


    if (flame) {

        flameElement.textContent =
            "DETECTED";

        flameElement.className =
            "value fire";

    }

    else {

        flameElement.textContent =
            "NORMAL";

        flameElement.className =
            "value normal";

    }



    /* =====================================================
       GAS
    ====================================================== */

    document
        .getElementById(
            `node${id}-gas`
        )
        .textContent =

        gas !== undefined
            ? gas
            : "--";



    /* =====================================================
       X POSITION
    ====================================================== */

    document
        .getElementById(
            `node${id}-x`
        )
        .textContent =

        x !== undefined
            ? Number(x).toFixed(2) + " m"
            : "--";



    /* =====================================================
       Y POSITION
    ====================================================== */

    document
        .getElementById(
            `node${id}-y`
        )
        .textContent =

        y !== undefined
            ? Number(y).toFixed(2) + " m"
            : "--";



    /* =====================================================
       ALERT
    ====================================================== */

    const alertElement =
        document
            .getElementById(
                `node${id}-alert`
            );


    const statusElement =
        document
            .getElementById(
                `node${id}-status`
            );


    const card =
        document
            .getElementById(
                `node-card-${id}`
            );


    const fireAlert =
        document
            .getElementById(
                `node${id}-fire-alert`
            );



    /* =====================================================
       ALERT ACTIVE
    ====================================================== */

    if (alert) {


        alertElement.textContent =
            "ALERT";


        alertElement.className =
            "value fire";


        statusElement.textContent =
            "ALERT";


        statusElement.className =
            "node-status alert";


        card.className =
            "card node-card alert";


        /* SHOW FIRE ALERT */

        fireAlert
            .classList
            .remove("hidden");


        fireAlert.textContent =
            `🔥 FIRE ALERT — NODE ${id}`;

    }


    /* =====================================================
       NO ALERT
    ====================================================== */

    else {


        alertElement.textContent =
            "NORMAL";


        alertElement.className =
            "value normal";


        statusElement.textContent =
            "OK";


        statusElement.className =
            "node-status";


        card.className =
            "card node-card";


        /* HIDE FIRE ALERT */

        fireAlert
            .classList
            .add("hidden");

    }

}



/* =========================================================
   ALERT SUMMARY
========================================================= */

function updateAlertSummary(

    data,

    alertCount

) {


    const summary =
        document
            .getElementById(
                "alert-summary"
            );


    /* =====================================================
       NO ALERTS
    ====================================================== */

    if (alertCount === 0) {

        summary.innerHTML =

            `<span class="alert-normal">
                ✓ No active alerts
             </span>`;

        return;

    }



    /* =====================================================
       BUILD ALERT MESSAGE
    ====================================================== */

    let messages = [];


    if (data.node1_alert) {

        messages.push(

            `Node 1: ${
                data.node1_flame
                    ? "FIRE DETECTED"
                    : "SENSOR ALERT"
            }`

        );

    }


    if (data.node2_alert) {

        messages.push(

            `Node 2: ${
                data.node2_flame
                    ? "FIRE DETECTED"
                    : "SENSOR ALERT"
            }`

        );

    }



    /* =====================================================
       DISPLAY ALERT
    ====================================================== */

    summary.innerHTML =

        `<span class="alert-active">
             ${messages.join(" | ")}
         </span>`;

}