#include <HardwareSerial.h>
#include <Ethernet.h>
#include <WiFiNINA.h>

// Set up the Ethernet or WiFi client
EthernetClient client;
//WiFiClient client;

// Set up the server and request path for ChatGPT
const char* server = "your-chatgpt-server.com";
const char* requestPath = "/api/generate_commands";

// Set up the target coordinates
int targetX = 0;
int targetY = 0;

// Set up the serial port for the Lidar
HardwareSerial LidarSerial(1);

void setup() {
  // Set up the serial port for debugging
  Serial.begin(9600);

  // Set up the Ethernet or WiFi connection
  //Ethernet.begin(mac, ip);
  //Serial.print("IP address: ");
  //Serial.println(Ethernet.localIP());
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up the serial port for the Lidar
  LidarSerial.begin(115200, SERIAL_8N1, 13, 12);
  
  // Wait for the Lidar to start up
  delay(1000);
}

void loop() {
  // Read the Lidar data
  String lidarData = readLidarData();

  // Preprocess the Lidar data
  String obstacles = preprocessLidarData(lidarData);

  // Generate natural language commands
  String commands = generateCommands(obstacles, targetX, targetY);

  // Send the HTTP requests to ChatGPT
  if (client.connect(server, 80)) {
    // Construct the HTTP request
    String httpRequest = "GET " + String(requestPath) + "?data=" + String(commands) + " HTTP/1.1\r\n" +
                         "Host: " + String(server) + "\r\n" +
                         "Connection: close\r\n" +
                         "\r\n";

    // Send the HTTP request
    client.print(httpRequest);
  } else {
    Serial.println("Connection failed");
  }

  // Wait for the response from ChatGPT
  String response = "";
  while (client.connected()) {
    if (client.available()) {
      response += client.readStringUntil('\r');
    }

    if (response.endsWith("\n")) {
      break;
    }

    delay(1);
  }

  // Parse the response and navigate the robot
  navigateRobot(response);

  // Check for user input to update the target position
  if (Serial.available()) {
    String userInput = Serial.readStringUntil('\n');
    int commaIndex = userInput.indexOf(',');
    if (commaIndex != -1) {
      targetX = userInput.substring(0, commaIndex).toInt();
      targetY = userInput.substring(commaIndex+1).toInt();
      Serial.print("New target position: ");
      Serial.print(targetX);
      Serial.print(",");
      Serial.println(targetY);
    }
  }
}

String readLidarData() {
  String data = "";
  while (LidarSerial.available()) {
    char c = LidarSerial.read();
    if (c == '\n') {
      break;
    }
    data += c;
  }
  return data;
}

String preprocessLidarData(String lidarData) {
  String obstacles = "";

  // Extract the obstacle data from the Lidar data
  int start = lidarData.indexOf("360,");
  int end = lidarData.indexOf("720,");
  String obstacleData = lidarData.substring(start, end);

  // Convert the obstacle data into an array of distances
  int distances[361];
  int commaIndex;
  for (int i = 0; i <= 360; i++) {
    commaIndex = obstacleData.indexOf(',');
    distances[i] = obstacleData.substring(0, commaIndex).toInt();
    obstacleData = obstacleData.substring(commaIndex + 1);
  }

  // Find the closest obstacles
  int closestRight = -1;
  int closestLeft = -1;
  for (int i = 0; i <= 180; i++) {
    if (distances[i] > 0 && (closestRight == -1 || distances[i] < distances[closestRight])) {
      closestRight = i;
    }
  }
  for (int i = 180; i <= 360; i++) {
    if (distances[i] > 0 && (closestLeft == -1 || distances[i] < distances[closestLeft])) {
      closestLeft = i;
    }
  }

  // Generate the obstacle string
  if (closestRight != -1 || closestLeft != -1) {
    if (closestRight != -1 && closestLeft != -1) {
      obstacles = "Obstacles to the right and left";
    }
    else if (closestRight != -1) {
      obstacles = "Obstacle to the right";
    }
    else {
      obstacles = "Obstacle to the left";
    }
  }

  return obstacles;
}


String generateCommands(String obstacles, int targetX, int targetY) {
  String commands = "";

  // If there are no obstacles, move towards the target
  if (obstacles == "") {
    commands = "Move to " + String(targetX) + "," + String(targetY);
  }
  // If there are obstacles, avoid them and move towards the target
  else {
    // Parse the obstacle information
    String obstacleList[10];
    int numObstacles = 0;
    int minDist = 1000;
    int closestObstacle = -1;
    int angle;
    int dist;

    int startIndex = obstacles.indexOf("360,");
    int endIndex = obstacles.indexOf("660,");
    obstacles = obstacles.substring(startIndex, endIndex);
    int commaIndex = obstacles.indexOf(',');
    obstacles = obstacles.substring(commaIndex+1);
    
    while (obstacles.indexOf(',') != -1) {
      commaIndex = obstacles.indexOf(',');
      obstacleList[numObstacles] = obstacles.substring(0, commaIndex);
      obstacles = obstacles.substring(commaIndex+1);
      commaIndex = obstacles.indexOf(',');
      angle = obstacles.substring(0, commaIndex).toInt();
      obstacles = obstacles.substring(commaIndex+1);
      commaIndex = obstacles.indexOf(',');
      dist = obstacles.substring(0, commaIndex).toInt();
      obstacles = obstacles.substring(commaIndex+1);

      if (dist < minDist) {
        minDist = dist;
        closestObstacle = numObstacles;
      }

      numObstacles++;
      if (numObstacles >= 10) {
        break;
      }
    }

    // Determine the direction of the closest obstacle
    String obstacleDirection;
    if (closestObstacle != -1) {
      if (angle < 90) {
        obstacleDirection = "to the right";
      } else if (angle > 270) {
        obstacleDirection = "to the left";
      } else {
        obstacleDirection = "in front";
      }
    }

    // Generate the avoidance commands
    if (closestObstacle != -1) {
      commands = "Avoid the obstacle " + obstacleDirection;
    }

    // Add the movement commands
    if (targetX != 0 || targetY != 0) {
      commands += " and move to " + String(targetX) + "," + String(targetY);
    }
  }

  return commands;
}

void navigateRobot(String response) {
  // Parse the response from ChatGPT and extract the movement commands
  String commands = "";
  int startIndex = response.indexOf("[[") + 2;
  int endIndex = response.indexOf("]]");
  if (startIndex != -1 && endIndex != -1) {
    commands = response.substring(startIndex, endIndex);
  }

  // Determine the movement based on the commands
  if (commands.indexOf("Move to") != -1) {
    // Extract the target position
    int commaIndex = commands.indexOf(',');
    int targetX = commands.substring(8, commaIndex).toInt();
    int targetY = commands.substring(commaIndex+1).toInt();

    // Move the robot towards the target position
    // For example, you could use the following code to move the robot forwards:
    // motor1.setSpeed(200);
    // motor1.run(FORWARD);
    // motor2.setSpeed(200);
    // motor2.run(FORWARD);
  }
  else if (commands.indexOf("Avoid the obstacle") != -1) {
    // Avoid the obstacle detected
    // For example, you could use the following code to turn the robot left:
    // motor1.setSpeed(200);
    // motor1.run(BACKWARD);
    // motor2.setSpeed(200);
    // motor2.run(FORWARD);
  }
}
