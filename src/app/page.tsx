"use client";

import { useState, useEffect } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";

export default function Home() {
  const [isRunning, setIsRunning] = useState(false);
  const [currentStep, setCurrentStep] = useState(0);
  const [robotPosition, setRobotPosition] = useState({ x: 50, y: 50 });
  const [targetPosition, setTargetPosition] = useState({ x: 200, y: 150 });
  const [slamProgress, setSlamProgress] = useState(0);
  const [localizationAccuracy, setLocalizationAccuracy] = useState(0);
  const [pathPlanningStatus, setPathPlanningStatus] = useState("Idle");

  const steps = [
    "Initializing ROS Nodes",
    "Starting Gazebo Simulation",
    "Launching SLAM (GMapping)",
    "Activating AMCL Localization", 
    "Starting Path Planners (A* + DWA)",
    "Enabling MoveIt! Integration",
    "Running Custom Sensor/Motor Nodes",
    "Navigation System Active"
  ];

  const rosNodes = [
    { name: "gmapping_node", status: "running", package: "slam_gmapping" },
    { name: "amcl_node", status: "running", package: "amcl_localization" },
    { name: "astar_planner", status: "running", package: "path_planning" },
    { name: "dwa_planner", status: "running", package: "path_planning" },
    { name: "moveit_node", status: "running", package: "moveit_integration" },
    { name: "sensor_processing", status: "running", package: "custom_nodes" },
    { name: "motor_control", status: "running", package: "custom_nodes" }
  ];

  useEffect(() => {
    if (isRunning) {
      const interval = setInterval(() => {
        setCurrentStep((prev) => {
          if (prev < steps.length - 1) {
            return prev + 1;
          }
          return prev;
        });
        
        setSlamProgress((prev) => Math.min(prev + 2, 100));
        setLocalizationAccuracy((prev) => Math.min(prev + 1.5, 99.8));
        
        // Simulate robot movement
        setRobotPosition((prev) => ({
          x: prev.x + (Math.random() - 0.5) * 4,
          y: prev.y + (Math.random() - 0.5) * 4
        }));
        
        if (Math.random() > 0.7) {
          setPathPlanningStatus(["Planning", "Executing", "Avoiding Obstacle", "Reached Waypoint"][Math.floor(Math.random() * 4)]);
        }
      }, 1000);

      return () => clearInterval(interval);
    }
  }, [isRunning, steps.length]);

  const startDemo = () => {
    setIsRunning(true);
    setCurrentStep(0);
    setSlamProgress(0);
    setLocalizationAccuracy(0);
    setPathPlanningStatus("Initializing");
  };

  const stopDemo = () => {
    setIsRunning(false);
    setCurrentStep(0);
    setSlamProgress(0);
    setLocalizationAccuracy(0);
    setPathPlanningStatus("Idle");
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-gray-900 to-black text-white p-8">
      <div className="max-w-7xl mx-auto">
        <div className="text-center mb-8">
          <h1 className="text-4xl font-bold mb-4">Autonomous Mobile Robot Navigation</h1>
          <p className="text-xl text-gray-300 mb-6">ROS-based Navigation System Demonstration</p>
          <div className="flex justify-center gap-4">
            <Button onClick={startDemo} disabled={isRunning} className="bg-green-600 hover:bg-green-700">
              Start Navigation Demo
            </Button>
            <Button onClick={stopDemo} disabled={!isRunning} variant="destructive">
              Stop Demo
            </Button>
          </div>
        </div>

        <Tabs defaultValue="overview" className="w-full">
          <TabsList className="grid w-full grid-cols-4 bg-gray-800">
            <TabsTrigger value="overview">System Overview</TabsTrigger>
            <TabsTrigger value="simulation">Live Simulation</TabsTrigger>
            <TabsTrigger value="nodes">ROS Nodes</TabsTrigger>
            <TabsTrigger value="metrics">Performance</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              <Card className="bg-gray-800 border-gray-700">
                <CardHeader>
                  <CardTitle className="text-green-400">SLAM (GMapping)</CardTitle>
                  <CardDescription>Simultaneous Localization and Mapping</CardDescription>
                </CardHeader>
                <CardContent>
                  <p className="text-sm text-gray-300">Real-time mapping using lidar data with GMapping algorithm</p>
                  <Progress value={slamProgress} className="mt-2" />
                  <p className="text-xs text-gray-400 mt-1">Mapping Progress: {slamProgress.toFixed(1)}%</p>
                </CardContent>
              </Card>

              <Card className="bg-gray-800 border-gray-700">
                <CardHeader>
                  <CardTitle className="text-blue-400">AMCL Localization</CardTitle>
                  <CardDescription>Adaptive Monte Carlo Localization</CardDescription>
                </CardHeader>
                <CardContent>
                  <p className="text-sm text-gray-300">Particle filter-based localization with high accuracy</p>
                  <Progress value={localizationAccuracy} className="mt-2" />
                  <p className="text-xs text-gray-400 mt-1">Accuracy: {localizationAccuracy.toFixed(1)}%</p>
                </CardContent>
              </Card>

              <Card className="bg-gray-800 border-gray-700">
                <CardHeader>
                  <CardTitle className="text-purple-400">Path Planning</CardTitle>
                  <CardDescription>A* Global + DWA Local Planning</CardDescription>
                </CardHeader>
                <CardContent>
                  <p className="text-sm text-gray-300">Collision-free navigation with 95% success rate</p>
                  <Badge variant="outline" className="mt-2">{pathPlanningStatus}</Badge>
                </CardContent>
              </Card>
            </div>

            <Card className="bg-gray-800 border-gray-700">
              <CardHeader>
                <CardTitle>System Initialization Progress</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-2">
                  {steps.map((step, index) => (
                    <div key={index} className={`flex items-center space-x-2 ${index <= currentStep ? 'text-green-400' : 'text-gray-500'}`}>
                      <div className={`w-3 h-3 rounded-full ${index <= currentStep ? 'bg-green-400' : 'bg-gray-600'}`} />
                      <span>{step}</span>
                      {index === currentStep && isRunning && <span className="text-yellow-400">...</span>}
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="simulation" className="space-y-6">
            <Card className="bg-gray-800 border-gray-700">
              <CardHeader>
                <CardTitle>Gazebo Simulation Environment</CardTitle>
                <CardDescription>TurtleBot3 in Indoor Environment</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="relative w-full h-96 bg-gray-900 border border-gray-600 rounded-lg overflow-hidden">
                  {/* Simulated environment */}
                  <div className="absolute inset-0">
                    {/* Walls */}
                    <div className="absolute top-4 left-4 w-80 h-2 bg-gray-600"></div>
                    <div className="absolute top-4 left-4 w-2 h-60 bg-gray-600"></div>
                    <div className="absolute bottom-4 left-4 w-80 h-2 bg-gray-600"></div>
                    <div className="absolute top-4 right-4 w-2 h-60 bg-gray-600"></div>
                    
                    {/* Obstacles */}
                    <div className="absolute top-20 left-20 w-8 h-8 bg-red-600 rounded"></div>
                    <div className="absolute top-40 left-60 w-12 h-6 bg-red-600 rounded"></div>
                    
                    {/* Robot */}
                    <div 
                      className="absolute w-4 h-4 bg-blue-500 rounded-full transition-all duration-1000"
                      style={{ 
                        left: `${robotPosition.x}px`, 
                        top: `${robotPosition.y}px` 
                      }}
                    >
                      <div className="absolute -inset-8 border border-blue-300 rounded-full opacity-30"></div>
                    </div>
                    
                    {/* Target */}
                    <div 
                      className="absolute w-4 h-4 bg-green-500 rounded-full"
                      style={{ 
                        left: `${targetPosition.x}px`, 
                        top: `${targetPosition.y}px` 
                      }}
                    >
                      <div className="absolute -inset-2 border border-green-300 rounded-full"></div>
                    </div>
                    
                    {/* Path visualization */}
                    <svg className="absolute inset-0 w-full h-full">
                      <path
                        d={`M ${robotPosition.x + 8} ${robotPosition.y + 8} Q ${(robotPosition.x + targetPosition.x) / 2} ${robotPosition.y - 20} ${targetPosition.x + 8} ${targetPosition.y + 8}`}
                        stroke="#10b981"
                        strokeWidth="2"
                        fill="none"
                        strokeDasharray="5,5"
                        opacity="0.7"
                      />
                    </svg>
                  </div>
                  
                  <div className="absolute bottom-2 left-2 text-xs text-gray-400">
                    <div>🔵 Robot Position: ({robotPosition.x.toFixed(0)}, {robotPosition.y.toFixed(0)})</div>
                    <div>🟢 Target: ({targetPosition.x}, {targetPosition.y})</div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="nodes" className="space-y-6">
            <Card className="bg-gray-800 border-gray-700">
              <CardHeader>
                <CardTitle>Active ROS Nodes</CardTitle>
                <CardDescription>Real-time status of navigation system components</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  {rosNodes.map((node, index) => (
                    <div key={index} className="flex items-center justify-between p-3 bg-gray-900 rounded-lg">
                      <div>
                        <div className="font-medium">{node.name}</div>
                        <div className="text-sm text-gray-400">{node.package}</div>
                      </div>
                      <Badge variant={node.status === 'running' ? 'default' : 'destructive'}>
                        {node.status}
                      </Badge>
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="metrics" className="space-y-6">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
              <Card className="bg-gray-800 border-gray-700">
                <CardHeader className="pb-2">
                  <CardTitle className="text-lg">Localization Error</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-green-400">
                    {"<0.1m"}
                  </div>
                  <p className="text-xs text-gray-400">Target achieved</p>
                </CardContent>
              </Card>

              <Card className="bg-gray-800 border-gray-700">
                <CardHeader className="pb-2">
                  <CardTitle className="text-lg">Success Rate</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-green-400">95%</div>
                  <p className="text-xs text-gray-400">Navigation success</p>
                </CardContent>
              </Card>

              <Card className="bg-gray-800 border-gray-700">
                <CardHeader className="pb-2">
                  <CardTitle className="text-lg">CPU Usage</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-yellow-400">
                    {isRunning ? Math.floor(Math.random() * 20 + 60) : 0}%
                  </div>
                  <p className="text-xs text-gray-400">System load</p>
                </CardContent>
              </Card>

              <Card className="bg-gray-800 border-gray-700">
                <CardHeader className="pb-2">
                  <CardTitle className="text-lg">Memory</CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="text-2xl font-bold text-blue-400">
                    {isRunning ? (2.1 + Math.random() * 0.5).toFixed(1) : 0.0}GB
                  </div>
                  <p className="text-xs text-gray-400">RAM usage</p>
                </CardContent>
              </Card>
            </div>

            <Card className="bg-gray-800 border-gray-700">
              <CardHeader>
                <CardTitle>System Architecture</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="text-sm space-y-2">
                  <div><strong>Technologies:</strong> ROS Melodic/Noetic, C++, Python, Gazebo, TurtleBot3</div>
                  <div><strong>SLAM:</strong> GMapping with Lidar-based real-time mapping</div>
                  <div><strong>Localization:</strong> AMCL (Adaptive Monte Carlo Localization)</div>
                  <div><strong>Planning:</strong> A* global planner + DWA local planner</div>
                  <div><strong>Motion Planning:</strong> MoveIt! integration</div>
                  <div><strong>Custom Nodes:</strong> Sensor processing and motor control</div>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>

        <div className="mt-8 text-center">
          <p className="text-gray-400">
            GitHub Repository: 
            <a 
              href="https://github.com/AyushVJha/autonomous-mobile-robot-navigation-using-ros" 
              className="text-blue-400 hover:text-blue-300 ml-2"
              target="_blank"
              rel="noopener noreferrer"
            >
              autonomous-mobile-robot-navigation-using-ros
            </a>
          </p>
        </div>
      </div>
    </div>
  );
}
