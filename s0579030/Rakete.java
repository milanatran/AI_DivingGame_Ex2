package s0579030;

import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.Random;

//import WeightedGraph.Mode;
import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

public class Rakete extends AI {
	private int pearlsFound;
	Point[] pearls = info.getScene().getPearl(); // Pearls in current level
	int numRays = 30; // Amount of rays
	int obstacleCheckpointAmount = 20;
	int viewFieldAngle = 90; // Angle of view field
	float fleeThreshold; // Distance to obstacle when starting to flee
	float screenRatio = info.getScene().getWidth() / info.getScene().getHeight();
	int widthDivision = 50;
	int heightDivision = (int) (widthDivision * screenRatio); // CHECK HERE IF ERROR
	boolean[][] freeSpace = new boolean[widthDivision][heightDivision];
	Path2D[] obstacles = info.getScene().getObstacles(); // Obstacles in level
	Vertex[] vertices = new Vertex[widthDivision * heightDivision];
	Vector2D halfTileVector = new Vector2D(info.getScene().getWidth() / widthDivision / 2, info.getScene().getHeight() / heightDivision / 2);

	public Rakete(Info info) {
		super(info);
		enlistForTournament(579030, 577618);
		pearlsFound = 0;
		
		// Sort pearls by x value with bubble sort
		for(int i = 0; i < pearls.length - 1; i++) {
			for(int j = 0; j < pearls.length - i - 1; j++) {
				if(pearls[j].getX() > pearls[j + 1].getX()) {
					Point temp = pearls[j];
					pearls[j] = pearls[j + 1];
					pearls[j + 1] = temp;
				}
			}
		}
		
		// Set pearl position and starting position
		Vector2D pearlPosition = new Vector2D((float) pearls[0].getX(), (float) pearls[0].getY());
		Vector2D startingPosition = new Vector2D((float) info.getX(), (float) info.getY());
		
		// Scan grid for obstacles and set free tiles to true in freeSpace
		for(int x = 0; x < widthDivision; x++) {
			for(int y = 0; y < heightDivision; y++) {
				Rectangle2D currentTile = new Rectangle2D.Float();
				currentTile.setFrame(x * info.getScene().getWidth() / widthDivision, y * info.getScene().getHeight() / heightDivision, info.getScene().getWidth() / widthDivision, info.getScene().getHeight() / heightDivision);
				
				// Check each obstacle if it intersects with current tile
				for(int obstacle = 0; obstacle < obstacles.length; obstacle++) {
					if(obstacles[obstacle].intersects(currentTile)) {
						freeSpace[x][y] = false;
						
						vertices[x + y * widthDivision] = null;
					}
					else {
						freeSpace[x][y] = true;
						
						// If tile is free, create new vertex for graph in the middle of the tile
						Vector2D vertexPosition = new Vector2D(x * info.getScene().getWidth() / widthDivision, y * info.getScene().getHeight() / heightDivision).addVector(halfTileVector);
						vertices[x + y * widthDivision] = new Vertex(vertexPosition, pearlPosition);
					}
				}
			}
		}
		
		// Set the neighbours for each vertex
		for(int vertex = 0; vertex < vertices.length; vertex++) {
			
			if(vertex % widthDivision == 0 && vertex < widthDivision) {
				//links oben
				vertices[vertex].setNeighbour(0, null); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, null); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			}
			
			else if((vertex % widthDivision) - (widthDivision -1) == 0 && vertex < widthDivision) {
				//rechts oben
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, null); //rechts
				vertices[vertex].setNeighbour(2, null); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			}
			
			else if(vertex % widthDivision == 0 && vertex + widthDivision > vertices.length) {
				//links unten
				vertices[vertex].setNeighbour(0, null); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, null); //unten
			}
			
			else if((vertex % widthDivision) - (widthDivision -1) == 0 && vertex + widthDivision > vertices.length) {
				//rechts unten
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, null); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, null); //unten
			}
			
			else if(vertex % widthDivision == 0) {
				//linker Nachbar = null
				vertices[vertex].setNeighbour(0, null); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			}
			
			else if(vertex < widthDivision) {
				//oberer Nachbar = null
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, null); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			} 
			
			else if(vertex + widthDivision > vertices.length) {
				//unterer Nachbar 
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, null); //unten
			}
			
			else if((vertex % widthDivision) - (widthDivision -1) == 0) {
				//rechts
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, null); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			}
			else {
				vertices[vertex].setNeighbour(0, vertices[vertex - 1]); //links
				vertices[vertex].setNeighbour(1, vertices[vertex + 1]); //rechts
				vertices[vertex].setNeighbour(2, vertices[vertex - widthDivision]); //oben
				vertices[vertex].setNeighbour(3, vertices[vertex + widthDivision]); //unten
			}
		}
		
		ArrayList<Vertex> path = aStarPathFinding(startingPosition, pearlPosition);
	}
	
	public ArrayList<Vertex> aStarPathFinding(Vector2D startingPosition, Vector2D pearlPosition) {
		// A star algorithm
		// Set all vertices of the graph to infinite distance with no previous node
		for(Vertex vertex: vertices) {
			vertex.setDistanceFromStartPosition(Double.POSITIVE_INFINITY);
			vertex.setPreviousVertex(null);
			vertex.setExplored(false);
		}
		
		// Add a queue for the next node with smallest distance
		Vertex currentVertex;
		PriorityQueue<Vertex> unexploredVertices = new PriorityQueue<Vertex>();
		
		// Find closest vertex to start position and to pearl position
		double startToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToStart = vertices[0];
		
		double pearlToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToPearl = vertices[0];
		
		// Check distance to start and pearl for each vertex
		for(int i = 0; i < vertices.length; i++) {
			Vertex vertexToMeasure = vertices[i];
			
			// Only check existing vertices
			if(vertexToMeasure != null) {
				double startDistanceToCurrentVertex = vertices[i].getLocation().subtractVector(startingPosition).getLength();
				double pearlDistanceToCurrentVertex = vertices[i].getLocation().subtractVector(pearlPosition).getLength();
				
				if(startDistanceToCurrentVertex < startToVertexDistance) {
					startToVertexDistance = startDistanceToCurrentVertex;
					closestVertexToStart = vertexToMeasure;
				}
				
				if(pearlDistanceToCurrentVertex < pearlToVertexDistance) {
					pearlToVertexDistance = pearlDistanceToCurrentVertex;
					closestVertexToPearl = vertexToMeasure;
				}
			}
		}
		
		// Add start node to the priority queue and set destination node
		unexploredVertices.add(closestVertexToStart);
		closestVertexToStart.setDistanceFromStartPosition(closestVertexToStart.getDistanceToEnd());
		closestVertexToStart.setPreviousVertex(null);
		Vertex destination = closestVertexToPearl;
		
		// For every neighbour of the current node update the distance.
		//set new previous node to current node.
		while(!destination.getExplored()) {
			//pull the nearest element out of the queue and get its neighbours.
			currentVertex = (Vertex) unexploredVertices.poll();
			Vertex[] neighbours = new Vertex[4];
			
			if (currentVertex != null) {
				neighbours = currentVertex.getNeighbours();
			}
			else {
				return null;
			}
			
			//look at all neighbours and check/update their distances.
			for(Vertex neighbour : neighbours) {
				if (!neighbour.getExplored()) {
					//if the neighbour doesnt have a distance yet, set it and queue it.
					if (neighbour.getDistanceFromStartPosition() == Double.POSITIVE_INFINITY) {
						neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
						neighbour.setPreviousVertex(currentVertex);
						unexploredVertices.add(neighbour);
					}
					//if it has a distance, just update it.
					else {
						neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
						neighbour.setPreviousVertex(currentVertex);
					}
						
				}
			}
			//set current node to explored so it wont be checked again.
			currentVertex.setExplored(true);
		}
		
		//backtrack the path from the destination to the start and return it as string.
		currentVertex = destination;
		ArrayList<Vertex> path = new ArrayList<>();
		
		while(currentVertex != null) {
			path.add(currentVertex);
			currentVertex = currentVertex.getPreviousVertex();
		}
		return path;
	}

	@Override
	public String getName() {
		return "Rakete";
	}

	@Override
	public Color getPrimaryColor() {
		return Color.YELLOW;
	}

	@Override
	public Color getSecondaryColor() {
		return Color.BLUE;
	}

	@Override
	public PlayerAction update() {
		// Get diver position
		double startX = info.getX();
		double startY = info.getY();
		Vector2D startVector = new Vector2D((float) startX, (float) startY);
		
		// Get pearl position
		double seekZielX = pearls[pearlsFound].getX();
		double seekZielY = pearls[pearlsFound].getY();
		Vector2D seekVector = new Vector2D((float) seekZielX, (float) seekZielY);
		
		float distanceToPearl = (int) Math.sqrt(Math.pow(startY - seekZielY, 2) + Math.pow(startX - seekZielX, 2));
		
		// Check if pearl was found
		if(info.getScore()!= pearlsFound) {
			pearls[pearlsFound] = null;
			pearlsFound++;
		}
		
		// Seek pearl
		Vector2D seekDirection = seekVector.subtractVector(startVector);
		seekDirection = seekDirection.normalize();
		
		// Distance to an obstacle for each ray
		int[] distanceToObstacle = new int[numRays];
		
		// Nearest obstacle point for each ray
		Point2D[] nearestObstaclePoints = new Point2D[numRays];
		
		int angleBetweenRays = viewFieldAngle/numRays;
		
		int distanceOfPoints = 4;
		
		// Check each ray
		for(int ray = 0; ray < numRays; ray++) {
			boolean obstacleFound = false;
			
			// Calculate ray direction and normalize it
			Vector2D rayDirection = new Vector2D();
			float rayRotation = (float) ((viewFieldAngle/2 - angleBetweenRays * ray) * Math.PI / 180);
			
			rayDirection.set(seekDirection.getX(), seekDirection.getY());
			rayDirection = rayDirection.rotate(rayRotation);
			rayDirection = rayDirection.normalize();
			
			// Check each point on ray
			for(int pointOnRay = 0; pointOnRay < obstacleCheckpointAmount && obstacleFound == false; pointOnRay++) {
				//System.out.println("Point: " + pointOnRay);
				// Calculate checkpoint on ray
				Point2D obstacleCheckpoint = startVector.addVector(rayDirection.multiplyVector(pointOnRay * distanceOfPoints)).convertToPoint();
				
				// Check for each obstacle if it contains point
				for(int obstacle = 0; obstacle < obstacles.length && obstacleFound == false; obstacle++) {
					if(obstacles[obstacle].contains(obstacleCheckpoint)) {
						// Store nearest point in obstacle and distance to obstacle
						distanceToObstacle[ray] = pointOnRay * distanceOfPoints;
						nearestObstaclePoints[ray] = obstacleCheckpoint;
						obstacleFound = true;
					}
					else {
						distanceToObstacle[ray] = Integer.MAX_VALUE;
					}
				}
			}
		}
		
		int closestDistance = Integer.MAX_VALUE; // Smallest distance to obstacle
		int closestRay = 0; // Closest ray to obstacle
		
		// Get closest ray to obstacle
		for(int ray = 0; ray < numRays; ray++) {
			if(distanceToObstacle[ray] < closestDistance) {
				closestDistance = distanceToObstacle[ray];
				closestRay = ray;
			}
		}
		
		// Calculate flee direction
		Vector2D fleeDirection;
		float fleeFromObstacleFactor = distanceOfPoints * obstacleCheckpointAmount;
		float seekPearlFactor = 5;
		
		if(closestDistance < fleeThreshold) {
			// Flee until further away
			fleeThreshold = Math.min(distanceOfPoints * obstacleCheckpointAmount, (fleeFromObstacleFactor / closestDistance) * distanceToPearl / seekPearlFactor);
			
			// Flee from closest obstacle point
			Vector2D fleeVector = new Vector2D((float)(nearestObstaclePoints[closestRay].getX()), (float)(nearestObstaclePoints[closestRay].getY()));
			
			fleeDirection = startVector.subtractVector(fleeVector);
			
			// Rotate flee vector randomly if its the opposite to flee
			if(fleeDirection == seekDirection.multiplyVector(-1)) {
				int randomAngle = (int) Math.floor(Math.random() * ((5) - (-5) + 1) + (-5));;
				fleeDirection = fleeDirection.rotate((float) (randomAngle * Math.PI/180));
			}
		} else {
			// Seek until close to obstacle
			fleeThreshold = 5;
			
			// Keep seeking if there is no obstacle
			fleeDirection = seekDirection;
		}
		
		fleeDirection = fleeDirection.normalize();
		seekDirection = seekDirection.normalize();
		
		// Combine seek and flee behavior
		Vector2D directionVector = seekDirection.multiplyVector(1f).addVector(fleeDirection.multiplyVector(fleeFromObstacleFactor / closestDistance));
		
		// Calculate direction radiant value
		float direction = (float) Math.atan2(directionVector.getY(), directionVector.getX());

		return new DivingAction(1, -direction);
	}

}

