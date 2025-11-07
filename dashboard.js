// Connect to local MQTT broker over WebSocket (port 9001)
const client = mqtt.connect("ws://172.20.10.13:9001");

// set defaults
document.getElementById("obstacleDetectedValue").innerText = "No";

client.on("connect", () => {
  document.getElementById("status").innerText = "Connected to MQTT Broker";
  client.subscribe("telemetry/#");
  console.log("Subscribed to telemetry/#");
});

client.on("message", (topic, message) => {
  console.log("Received:", topic, message.toString());

  let data;
  try {
    data = JSON.parse(message.toString());
  } catch (err) {
    console.error("Invalid JSON received:", message.toString());
    return;
  }

  switch (topic) {
    // ------------------ Motion & IMU ------------------
    case "telemetry/speed":
      if (data.speed !== undefined)
        document.getElementById("speedValue").innerText = data.speed.toFixed(2);
      break;

    case "telemetry/distance":
      if (data.distance !== undefined)
        document.getElementById("distValue").innerText = data.distance.toFixed(2);
      break;

    case "telemetry/imu/raw":
      if (data.raw_pitch !== undefined)
        document.getElementById("raw_pitch").innerText = data.raw_pitch.toFixed(2);
      if (data.raw_roll !== undefined)
        document.getElementById("raw_roll").innerText = data.raw_roll.toFixed(2);
      if (data.raw_heading !== undefined)
        document.getElementById("raw_heading").innerText = data.raw_heading.toFixed(2);
      break;

    case "telemetry/imu/filtered":
      if (data.pitch !== undefined)
        document.getElementById("filtered_pitch").innerText = data.pitch.toFixed(2);
      if (data.roll !== undefined)
        document.getElementById("filtered_roll").innerText = data.roll.toFixed(2);
      if (data.heading !== undefined)
        document.getElementById("filtered_heading").innerText = data.heading.toFixed(2);
      break;

    // ------------------ Line Following + Barcode ------------------
    case "telemetry/barcode":
      document.getElementById("barcodeValue").innerText = data.barcode || "--";
      break;


    case "telemetry/line":
      document.getElementById("stateValue").innerText = data.state || "--";
      document.getElementById("lineEventValue").innerText = data.line_event || "--";
      break;
      
    // case "telemetry/state":
    //   document.getElementById("stateValue").innerText = data.state || "--";
    //   break;
    
    // case "telemetry/lineEvent":
    //   if (data.line_event !== undefined)
    //     document.getElementById("lineEventValue").innerText = data.line_event;
    //   break;

    // ------------------ Obstacle Detection ------------------
    case "telemetry/obstacle":
      if (data.distance !== undefined)
        document.getElementById("objdistValue").innerText = data.distance.toFixed(2);
      else
        document.getElementById("objdistValue").innerText = "--";

      if (data.clearance !== undefined)
        document.getElementById("clearValue").innerText = data.clearance;
      else
        document.getElementById("clearValue").innerText = "--";

      if (data.recovery_status !== undefined)
        document.getElementById("recovstatusValue").innerText = data.recovery_status;
      else
        document.getElementById("recovstatusValue").innerText = "--";

      if (data.obstacle !== undefined)
        document.getElementById("obstacleDetectedValue").innerText = data.obstacle;
      else
        document.getElementById("obstacleDetectedValue").innerText = "No";

      if (data.obstacle === "Detected") {
        if (data.obstacle_width !== undefined)
          document.getElementById("ObstaclewidthValue").innerText = data.obstacle_width;
        if (data.left_angle !== undefined)
          document.getElementById("leftAngleValue").innerText = data.left_angle.toFixed(1);
        if (data.right_angle !== undefined)
          document.getElementById("rightAngleValue").innerText = data.right_angle.toFixed(1);
      } else {
        // Clear values if no obstacle
        document.getElementById("ObstaclewidthValue").innerText = "--";
        document.getElementById("leftAngleValue").innerText = "--";
        document.getElementById("rightAngleValue").innerText = "--";
      }
    break;

    default:
      console.log("Unhandled topic:", topic);
  }
});
