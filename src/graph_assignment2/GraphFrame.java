/*
 * S04 - Graph Demo Application
 * @author WIN11
 */
package graph_assignment2;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.filechooser.FileNameExtensionFilter;

/**
 * Main frame for the Graph Demo application, providing a GUI for graph
 * operations.
 *
 * @author WIN11
 */
public class GraphFrame extends JFrame {

    private final GraphPanel graphPanel;
    private final JTextArea infoTextArea;
    private final List<Vertex> vertices = new ArrayList<>();
    private final List<Edge> edges = new ArrayList<>();
    private Vertex selectedVertex = null;

    /**
     * Constructs the GraphFrame, initializing the GUI components and event
     * listeners.
     */
    public GraphFrame() {
        setTitle("Undirected Graph Demo v1");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new BorderLayout());
        getContentPane().setBackground(new Color(240, 240, 240)); // Light gray background

        // Menu Bar
        JMenuBar menuBar = new JMenuBar();
        menuBar.setBackground(new Color(60, 63, 65)); // Dark gray menu bar
        menuBar.setForeground(Color.WHITE); // White text for menu
        JMenu fileMenu = new JMenu("File");
        fileMenu.setForeground(Color.WHITE);
        JMenuItem openItem = new JMenuItem("Open");
        JMenuItem saveItem = new JMenuItem("Save as...");
        JMenuItem clearItem = new JMenuItem("Clear");
        JMenuItem exitItem = new JMenuItem("Exit");
        fileMenu.add(openItem);
        fileMenu.add(saveItem);
        fileMenu.add(clearItem);
        fileMenu.add(exitItem);
        menuBar.add(fileMenu);

        JMenu graphMenu = new JMenu("Graph");
        graphMenu.setForeground(Color.WHITE);
        JMenuItem bfsItem = new JMenuItem("BFS");
        JMenuItem dfsItem = new JMenuItem("DFS");
        graphMenu.add(bfsItem);
        graphMenu.add(dfsItem);
        menuBar.add(graphMenu);

        JMenu algoMenu = new JMenu("Algorithm");
        algoMenu.setForeground(Color.WHITE);
        JMenuItem shortestPathItem = new JMenuItem("Find shortest path");
        JMenuItem mstItem = new JMenuItem("Find Minimum Spanning Tree");
        algoMenu.add(shortestPathItem);
        algoMenu.add(mstItem);
        menuBar.add(algoMenu);

        JMenu helpMenu = new JMenu("Help");
        helpMenu.setForeground(Color.WHITE);
        JMenuItem docsItem = new JMenuItem("Docs");
        JMenuItem aboutItem = new JMenuItem("About");
        helpMenu.add(docsItem);
        helpMenu.add(aboutItem);
        menuBar.add(helpMenu);

        setJMenuBar(menuBar);

        // Left Panel
        JPanel leftPanel = new JPanel();
        leftPanel.setLayout(new BoxLayout(leftPanel, BoxLayout.Y_AXIS));
        leftPanel.setBorder(BorderFactory.createTitledBorder(
                BorderFactory.createLineBorder(new Color(60, 63, 65)),
                "Graph's Information",
                javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION,
                javax.swing.border.TitledBorder.DEFAULT_POSITION,
                new java.awt.Font("Arial", java.awt.Font.BOLD, 12)));
        leftPanel.setBackground(new Color(245, 245, 245)); // Slightly lighter background
        leftPanel.setPreferredSize(new java.awt.Dimension(200, 600)); // Fixed width for balance

        infoTextArea = new JTextArea(10, 20);
        infoTextArea.setEditable(false);
        infoTextArea.setFont(new java.awt.Font("Arial", java.awt.Font.PLAIN, 12)); // Consistent font
        infoTextArea.setBackground(new Color(255, 255, 255)); // White text area
        infoTextArea.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5)); // Padding
        JScrollPane scrollPane = new JScrollPane(infoTextArea);
        leftPanel.add(scrollPane);

        JButton matrixButton = new JButton("Matrix");
        matrixButton.setFont(new java.awt.Font("Arial", java.awt.Font.PLAIN, 12));
        matrixButton.setBackground(new Color(100, 149, 237)); // Cornflower blue
        matrixButton.setForeground(Color.WHITE);
        matrixButton.setBorder(BorderFactory.createEmptyBorder(5, 10, 5, 10)); // Padding
        matrixButton.setFocusPainted(false);
        JButton listButton = new JButton("List");
        listButton.setFont(new java.awt.Font("Arial", java.awt.Font.PLAIN, 12));
        listButton.setBackground(new Color(100, 149, 237));
        listButton.setForeground(Color.WHITE);
        listButton.setBorder(BorderFactory.createEmptyBorder(5, 10, 5, 10));
        listButton.setFocusPainted(false);
        JPanel buttonPanel = new JPanel();
        buttonPanel.setBackground(new Color(245, 245, 245));
        buttonPanel.add(matrixButton);
        buttonPanel.add(listButton);
        leftPanel.add(buttonPanel);

        // Graph Panel
        graphPanel = new GraphPanel();
        graphPanel.setBorder(BorderFactory.createLineBorder(new Color(60, 63, 65)));
        graphPanel.setBackground(new Color(230, 230, 230)); // Subtle gray background

        // Main Layout
        add(leftPanel, BorderLayout.WEST);
        add(graphPanel, BorderLayout.CENTER);

        pack();
        setSize(800, 600);
        setLocationRelativeTo(null);

        // Action Listeners
        exitItem.addActionListener(e -> System.exit(0));

        // Add file operation listeners
        openItem.addActionListener(e -> openGraph());
        saveItem.addActionListener(e -> saveGraph());

        clearItem.addActionListener(e -> {
            vertices.clear();
            edges.clear();
            graphPanel.clearHighlight();
            updateInfo();
            graphPanel.repaint();
        });

        matrixButton.addActionListener(e -> showMatrix());
        listButton.addActionListener(e -> showAdjacencyList());

        bfsItem.addActionListener(e -> runBFS());
        dfsItem.addActionListener(e -> runDFS());
        shortestPathItem.addActionListener(e -> findShortestPath());
        mstItem.addActionListener(e -> findMST());

        aboutItem.addActionListener(e -> JOptionPane.showMessageDialog(this,
                "Graph Demo v1.0\n"
                        + "Developed by Group 2 of FPT University Can Tho Campus Students Majored In Information Assurance (IA1902 Class)\n"
                        + "Members:\n"
                        + "   - Nguyen Hai Huy (Leader)\n"
                        + "   - Le Vo Phuoc Sang\n"
                        + "   - Nguyen Hai Ha\n"
                        + "   - Tran Thi Thuy Quyen\n"
                        + "   - Nguyen Phuc Bao\n"
                        + "   - Huynh Kien Minh",
                "About",
                JOptionPane.INFORMATION_MESSAGE));

        docsItem.addActionListener(e -> showInstructions());
    }

    /**
     * Displays user instructions for interacting with the graph application.
     */
    private void showInstructions() {
        String instructions = "Graph Demo v1.0 User Instructions:\n\n"
                + "1. Add Vertex: Ctrl+Click on the graph panel.\n"
                + "2. Delete Vertex: Right-click on a vertex.\n"
                + "3. Move Vertex: Click and drag a vertex to a new location.\n" // <-- ADDED
                + "4. Add Edge: Click one vertex, then another. Enter a weight when prompted.\n"
                + "5. File Menu:\n"
                + "   - Open: Load a graph (not implemented).\n"
                + "   - Save as: Save the graph (not implemented).\n"
                + "   - Clear: Remove all vertices and edges.\n"
                + "   - Exit: Close the application.\n"
                + "6. Graph Menu:\n"
                + "   - BFS: Run Breadth-First Search from a specified vertex.\n"
                + "   - DFS: Run Depth-First Search from a specified vertex.\n"
                + "7. Algorithm Menu:\n"
                + "   - Find shortest path: Find the shortest path using Dijkstra's algorithm.\n"
                + "   - Find Minimum Spanning Tree: Find the MST using Prim's algorithm.\n"
                + "8. Graph Information Panel:\n"
                + "   - Matrix: Display the adjacency matrix.\n"
                + "   - List: Display the adjacency list.\n"
                + "9. Visual Feedback:\n"
                + "   - Vertices are orange-red circles with white index numbers.\n"
                + "   - Edges show weights at their midpoints in dark gray.\n"
                + "   - MST edges are highlighted in gold.";
        JOptionPane.showMessageDialog(this, instructions, "User Instructions",
                JOptionPane.INFORMATION_MESSAGE);
    }

    /**
     * Updates the information panel with the current graph representation.
     */
    private void updateInfo() {
        showMatrix();
    }

    /**
     * Displays the adjacency matrix of the graph in the infoTextArea.
     */
    private void showMatrix() {
        infoTextArea.setText(vertices.size() + "\n");
        int n = vertices.size();
        int[][] matrix = new int[n][n];
        for (Edge edge : edges) {
            int u = vertices.indexOf(edge.u);
            int v = vertices.indexOf(edge.v);
            if (u != -1 && v != -1) { // Ensure vertices exist
                matrix[u][v] = edge.weight;
                matrix[v][u] = edge.weight;
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                infoTextArea.append(matrix[i][j] + " ");
            }
            infoTextArea.append("\n");
        }
    }

    /**
     * Displays the adjacency list of the graph in the infoTextArea.
     */
    private void showAdjacencyList() {
        infoTextArea.setText("Adjacency List:\n");
        for (int i = 0; i < vertices.size(); i++) {
            infoTextArea.append(i + ": ");
            for (Edge edge : edges) {
                if (edge.u == vertices.get(i)) {
                    infoTextArea.append(vertices.indexOf(edge.v) + "(" + edge.weight + ") ");
                } else if (edge.v == vertices.get(i)) {
                    infoTextArea.append(vertices.indexOf(edge.u) + "(" + edge.weight + ") ");
                }
            }
            infoTextArea.append("\n");
        }
    }

    /**
     * Runs Breadth-First Search starting from a user-specified vertex.
     */
    private void runBFS() {
        if (vertices.isEmpty()) {
            return;
        }
        String startNodeStr = JOptionPane.showInputDialog(this, "Enter start vertex index:");
        try {
            int startNode = Integer.parseInt(startNodeStr);
            if (startNode < 0 || startNode >= vertices.size()) {
                JOptionPane.showMessageDialog(this, "Invalid vertex index.", "Error", JOptionPane.ERROR_MESSAGE);
                return;
            }

            StringBuilder bfsResult = new StringBuilder("BFS from vertex " + startNode + ": ");
            Queue<Vertex> queue = new LinkedList<>();
            boolean[] visited = new boolean[vertices.size()];
            List<Edge> bfsEdges = new ArrayList<>(); // Track BFS tree edges

            queue.add(vertices.get(startNode));
            visited[startNode] = true;

            while (!queue.isEmpty()) {
                Vertex u = queue.poll();
                bfsResult.append(vertices.indexOf(u)).append(" ");

                for (Edge edge : edges) {
                    if (edge.u == u) {
                        int vIndex = vertices.indexOf(edge.v);
                        if (!visited[vIndex]) {
                            visited[vIndex] = true;
                            queue.add(edge.v);
                            bfsEdges.add(edge); // Add edge to BFS tree
                        }
                    } else if (edge.v == u) {
                        int uIndex = vertices.indexOf(edge.u);
                        if (!visited[uIndex]) {
                            visited[uIndex] = true;
                            queue.add(edge.u);
                            bfsEdges.add(edge); // Add edge to BFS tree
                        }
                    }
                }
            }
            infoTextArea.setText(bfsResult.toString());
            graphPanel.highlightBFS(bfsEdges); // Highlight BFS tree
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Invalid input. Please enter an integer.", "Error", JOptionPane.ERROR_MESSAGE);
        }
    }

    /**
     * Runs Depth-First Search starting from a user-specified vertex.
     */
    private void runDFS() {
        if (vertices.isEmpty()) {
            return;
        }
        String startNodeStr = JOptionPane.showInputDialog(this, "Enter start vertex index:");
        try {
            int startNode = Integer.parseInt(startNodeStr);
            if (startNode < 0 || startNode >= vertices.size()) {
                JOptionPane.showMessageDialog(this, "Invalid vertex index.", "Error", JOptionPane.ERROR_MESSAGE);
                return;
            }

            StringBuilder dfsResult = new StringBuilder("DFS from vertex " + startNode + ": ");
            boolean[] visited = new boolean[vertices.size()];
            List<Edge> dfsEdges = new ArrayList<>(); // Track DFS tree edges
            dfsUtil(vertices.get(startNode), visited, dfsResult, dfsEdges);
            infoTextArea.setText(dfsResult.toString());
            graphPanel.highlightDFS(dfsEdges); // Highlight DFS tree
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Invalid input. Please enter an integer.", "Error", JOptionPane.ERROR_MESSAGE);
        }
    }

    /**
     * Utility method for DFS traversal.
     *
     * @param v Current vertex
     * @param visited Array tracking visited vertices
     * @param result StringBuilder to store DFS result
     * @param dfsEdges List to store DFS tree edges
     */
    private void dfsUtil(Vertex v, boolean[] visited, StringBuilder result, List<Edge> dfsEdges) {
        int vIndex = vertices.indexOf(v);
        visited[vIndex] = true;
        result.append(vIndex).append(" ");

        for (Edge edge : edges) {
            if (edge.u == v) {
                int neighborIndex = vertices.indexOf(edge.v);
                if (!visited[neighborIndex]) {
                    dfsEdges.add(edge); // Add edge to DFS tree
                    dfsUtil(edge.v, visited, result, dfsEdges);
                }
            } else if (edge.v == v) {
                int neighborIndex = vertices.indexOf(edge.u);
                if (!visited[neighborIndex]) {
                    dfsEdges.add(edge); // Add edge to DFS tree
                    dfsUtil(edge.u, visited, result, dfsEdges);
                }
            }
        }
    }

    /**
     * Finds the shortest path between two vertices using Dijkstra's algorithm.
     */
    private void findShortestPath() {
        graphPanel.clearHighlight();
        if (vertices.size() < 2) {
            JOptionPane.showMessageDialog(this, "Need at least 2 vertices.", "Error", JOptionPane.ERROR_MESSAGE);
            return;
        }
        String startNodeStr = JOptionPane.showInputDialog(this, "Enter start vertex index:");
        String endNodeStr = JOptionPane.showInputDialog(this, "Enter end vertex index:");
        try {
            int startNodeIdx = Integer.parseInt(startNodeStr);
            int endNodeIdx = Integer.parseInt(endNodeStr);
            int n = vertices.size();

            if (startNodeIdx < 0 || startNodeIdx >= n || endNodeIdx < 0 || endNodeIdx >= n) {
                JOptionPane.showMessageDialog(this, "Invalid vertex index.", "Error", JOptionPane.ERROR_MESSAGE);
                return;
            }

            int[][] graph = new int[n][n];
            for (Edge edge : edges) {
                int u = vertices.indexOf(edge.u);
                int v = vertices.indexOf(edge.v);
                graph[u][v] = edge.weight;
                graph[v][u] = edge.weight;
            }

            int[] dist = new int[n];
            int[] prev = new int[n];
            boolean[] sptSet = new boolean[n];

            Arrays.fill(dist, Integer.MAX_VALUE);
            Arrays.fill(prev, -1);
            dist[startNodeIdx] = 0;

            for (int count = 0; count < n - 1; count++) {
                int u = -1;
                int min = Integer.MAX_VALUE;
                for (int v = 0; v < n; v++) {
                    if (!sptSet[v] && dist[v] <= min) {
                        min = dist[v];
                        u = v;
                    }
                }

                if (u == -1) {
                    continue;
                }

                sptSet[u] = true;

                for (int v = 0; v < n; v++) {
                    if (!sptSet[v] && graph[u][v] != 0 && dist[u] != Integer.MAX_VALUE && dist[u] + graph[u][v] < dist[v]) {
                        dist[v] = dist[u] + graph[u][v];
                        prev[v] = u;
                    }
                }
            }

            if (dist[endNodeIdx] == Integer.MAX_VALUE) {
                infoTextArea.setText("No path from vertex " + startNodeIdx + " to " + endNodeIdx);
            } else {
                StringBuilder pathStr = new StringBuilder();
                List<Integer> pathVertices = new ArrayList<>();
                int current = endNodeIdx;
                while (current != -1) {
                    pathVertices.add(0, current);
                    pathStr.insert(0, " -> " + current);
                    current = prev[current];
                }
                pathStr.delete(0, 4);
                infoTextArea.setText("The length of the shortest path from " + startNodeIdx + " to " + endNodeIdx + " is " + dist[endNodeIdx] + "\n" + pathStr.toString());
                
                // Highlight shortest path
                graphPanel.highlightShortestPath(pathVertices);
            }
        } catch (NumberFormatException e) {
            JOptionPane.showMessageDialog(this, "Invalid input. Please enter an integer.", "Error", JOptionPane.ERROR_MESSAGE);
        }
    }

    /**
     * Finds the Minimum Spanning Tree using Prim's algorithm.
     */
    private void findMST() {
        if (vertices.isEmpty()) {
            JOptionPane.showMessageDialog(this, "Graph is empty.", "Error", JOptionPane.ERROR_MESSAGE);
            return;
        }
        int n = vertices.size();
        int[][] graph = new int[n][n];
        for (Edge edge : edges) {
            int u = vertices.indexOf(edge.u);
            int v = vertices.indexOf(edge.v);
            graph[u][v] = edge.weight;
            graph[v][u] = edge.weight;
        }

        int[] parent = new int[n];
        int[] key = new int[n];
        boolean[] mstSet = new boolean[n];

        Arrays.fill(key, Integer.MAX_VALUE);
        key[0] = 0;
        parent[0] = -1;

        for (int count = 0; count < n - 1; count++) {
            int u = -1;
            int min = Integer.MAX_VALUE;
            for (int v = 0; v < n; v++) {
                if (!mstSet[v] && key[v] < min) {
                    min = key[v];
                    u = v;
                }
            }

            if (u == -1) {
                continue;
            }

            mstSet[u] = true;

            for (int v = 0; v < n; v++) {
                if (graph[u][v] != 0 && !mstSet[v] && graph[u][v] < key[v]) {
                    parent[v] = u;
                    key[v] = graph[u][v];
                }
            }
        }

        int totalWeight = 0;
        for (int i = 1; i < n; i++) {
            if (parent[i] != -1) {
                totalWeight += graph[i][parent[i]];
            }
        }
        infoTextArea.setText("The minimum spanning tree weight is " + totalWeight);
        graphPanel.highlightMst(parent);
    }

    /**
     * Handles the logic for deleting a vertex after user confirmation.
     *
     * @param vertexToDelete The vertex to be deleted.
     */
    private void handleVertexDeletion(Vertex vertexToDelete) {
        int vertexIndex = vertices.indexOf(vertexToDelete);
        int choice = JOptionPane.showConfirmDialog(
                this,
                "Are you sure you want to delete vertex " + vertexIndex
                        + "?\nThis will also remove all connected edges.",
                "Confirm Deletion",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.WARNING_MESSAGE);

        if (choice == JOptionPane.YES_OPTION) {
            // Remove all edges connected to this vertex
            edges.removeIf(edge -> edge.u == vertexToDelete || edge.v == vertexToDelete);

            // Remove the vertex itself
            vertices.remove(vertexToDelete);

            // If the selected vertex was the one being deleted, clear the selection
            if (selectedVertex == vertexToDelete) {
                selectedVertex = null;
            }

            // Update UI
            updateInfo();
            graphPanel.clearHighlight(); // Clear any algorithm highlights
            graphPanel.repaint();
        }
    }

    /**
     * Handles the logic for deleting an edge after user confirmation.
     *
     * @param edgeToDelete The edge to be deleted.
     */
    private void handleEdgeDeletion(Edge edgeToDelete) {
        int u = vertices.indexOf(edgeToDelete.u);
        int v = vertices.indexOf(edgeToDelete.v);
        
        int choice = JOptionPane.showConfirmDialog(
                this,
                "Are you sure you want to delete the edge between vertex " + u 
                + " and vertex " + v + " (weight: " + edgeToDelete.weight + ")?",
                "Confirm Edge Deletion",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.WARNING_MESSAGE);

        if (choice == JOptionPane.YES_OPTION) {
            // Remove the edge
            edges.remove(edgeToDelete);

            // Update UI
            updateInfo();
            graphPanel.clearHighlight(); // Clear any algorithm highlights
            graphPanel.repaint();
        }
    }

    /**
     * Represents a vertex in the graph with x, y coordinates.
     */
    class Vertex {

        int x, y;

        Vertex(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Represents an edge in the graph with two vertices and a weight.
     */
    class Edge {

        Vertex u, v;
        int weight;

        Edge(Vertex u, Vertex v, int weight) {
            this.u = u;
            this.v = v;
            this.weight = weight;
        }
    }

    /**
     * Custom panel for drawing the graph.
     */
    class GraphPanel extends JPanel {

        private int[] mstParent = null;
        private List<Edge> bfsEdges = null;
        private List<Edge> dfsEdges = null;
        private List<Integer> shortestPathVertices = null;

        // --- NEW: State for dragging vertices ---
        private Vertex vertexBeingDragged = null;
        private boolean wasDragged = false;

        public GraphPanel() {
            setBackground(new Color(230, 230, 230));

            MouseAdapter mouseHandler = new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
                    wasDragged = false; // Reset drag flag
                    vertexBeingDragged = getVertexAt(e.getX(), e.getY());
                }

                @Override
                public void mouseDragged(MouseEvent e) {
                    if (vertexBeingDragged != null) {
                        wasDragged = true; // A drag has occurred
                        vertexBeingDragged.x = e.getX();
                        vertexBeingDragged.y = e.getY();
                        repaint();
                    }
                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    vertexBeingDragged = null;
                }

                @Override
                public void mouseClicked(MouseEvent e) {
                    if (wasDragged) {
                        return; // Do not process a click if it was the end of a drag
                    }

                    if (javax.swing.SwingUtilities.isRightMouseButton(e)) {
                        // Check if right-click is on an edge
                        Edge edgeToDelete = getEdgeAt(e.getX(), e.getY());
                        if (edgeToDelete != null) {
                            handleEdgeDeletion(edgeToDelete);
                            return;
                        }
                        
                        // If not on edge, check for vertex deletion
                        Vertex vertexToDelete = getVertexAt(e.getX(), e.getY());
                        if (vertexToDelete != null) {
                            handleVertexDeletion(vertexToDelete);
                        }
                        return;
                    }

                    clearHighlight();
                    if (e.isControlDown()) {
                        vertices.add(new Vertex(e.getX(), e.getY()));
                    } else {
                        Vertex clickedVertex = getVertexAt(e.getX(), e.getY());
                        if (clickedVertex != null) {
                            if (selectedVertex == null) {
                                selectedVertex = clickedVertex;
                            } else {
                                if (selectedVertex != clickedVertex) {
                                    String weightStr = JOptionPane.showInputDialog(GraphFrame.this,
                                            "Enter weight for edge:");
                                    if (weightStr != null) {
                                        try {
                                            int weight = Integer.parseInt(weightStr);
                                            Edge existingEdge = null;
                                            for (Edge edge : edges) {
                                                if ((edge.u == selectedVertex && edge.v == clickedVertex)
                                                        || (edge.u == clickedVertex && edge.v == selectedVertex)) {
                                                    existingEdge = edge;
                                                    break;
                                                }
                                            }
                                            if (existingEdge != null) {
                                                existingEdge.weight = weight;
                                            } else {
                                                edges.add(new Edge(selectedVertex, clickedVertex, weight));
                                            }
                                        } catch (NumberFormatException ex) {
                                            JOptionPane.showMessageDialog(GraphFrame.this, "Invalid weight.", "Error",
                                                    JOptionPane.ERROR_MESSAGE);
                                        }
                                    }
                                }
                                selectedVertex = null;
                            }
                        } else {
                            selectedVertex = null;
                        }
                    }
                    updateInfo();
                    repaint();
                }
            };

            addMouseListener(mouseHandler);
            addMouseMotionListener(mouseHandler);
        }

        /**
         * Highlights the BFS tree edges in the graph.
         *
         * @param edges List of BFS tree edges
         */
        public void highlightBFS(List<Edge> edges) {
            clearHighlight();
            this.bfsEdges = new ArrayList<>(edges);
            repaint();
        }

        /**
         * Highlights the DFS tree edges in the graph.
         *
         * @param edges List of DFS tree edges
         */
        public void highlightDFS(List<Edge> edges) {
            clearHighlight();
            this.dfsEdges = new ArrayList<>(edges);
            repaint();
        }

        /**
         * Highlights the shortest path in the graph.
         *
         * @param pathVertices List of vertex indices in the shortest path
         */
        public void highlightShortestPath(List<Integer> pathVertices) {
            clearHighlight();
            this.shortestPathVertices = new ArrayList<>(pathVertices);
            repaint();
        }

        /**
         * Highlights the MST edges in the graph.
         *
         * @param parent Array containing MST parent information
         */
        public void highlightMst(int[] parent) {
            clearHighlight();
            this.mstParent = parent;
            repaint();
        }

        /**
         * Clears any highlighted edges or paths.
         */
        public void clearHighlight() {
            this.mstParent = null;
            this.bfsEdges = null;
            this.dfsEdges = null;
            this.shortestPathVertices = null;
            repaint();
        }

        /**
         * Paints the graph components (vertices, edges, and highlights).
         *
         * @param g Graphics object for drawing
         */
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);

            // Draw regular edges
            g.setColor(new Color(100, 100, 100)); // Darker gray for edges
            for (Edge edge : edges) {
                g.drawLine(edge.u.x, edge.u.y, edge.v.x, edge.v.y);
                g.setColor(new Color(50, 50, 50)); // Darker for weight text
                g.setFont(new java.awt.Font("Arial", java.awt.Font.BOLD, 12));
                g.drawString(String.valueOf(edge.weight), (edge.u.x + edge.v.x) / 2, (edge.u.y + edge.v.y) / 2);
                g.setColor(new Color(100, 100, 100)); // Reset for next edge
            }

            // Draw BFS tree edges
            if (bfsEdges != null) {
                g.setColor(new Color(0, 255, 0)); // Green for BFS
                for (Edge edge : bfsEdges) {
                    g.drawLine(edge.u.x, edge.u.y, edge.v.x, edge.v.y);
                }
            }

            // Draw DFS tree edges
            if (dfsEdges != null) {
                g.setColor(new Color(255, 0, 255)); // Magenta for DFS
                for (Edge edge : dfsEdges) {
                    g.drawLine(edge.u.x, edge.u.y, edge.v.x, edge.v.y);
                }
            }

            // Draw shortest path
            if (shortestPathVertices != null && shortestPathVertices.size() > 1) {
                g.setColor(new Color(255, 0, 0)); // Red for shortest path
                for (int i = 0; i < shortestPathVertices.size() - 1; i++) {
                    Vertex u = vertices.get(shortestPathVertices.get(i));
                    Vertex v = vertices.get(shortestPathVertices.get(i + 1));
                    g.drawLine(u.x, u.y, v.x, v.y);
                }
            }

            // Draw MST edges
            if (mstParent != null) {
                g.setColor(new Color(255, 215, 0)); // Gold for MST
                for (int i = 1; i < vertices.size(); i++) {
                    if (mstParent[i] != -1) {
                        Vertex u = vertices.get(mstParent[i]);
                        Vertex v = vertices.get(i);
                        g.drawLine(u.x, u.y, v.x, v.y);
                    }
                }
            }

            // Draw vertices
            for (int i = 0; i < vertices.size(); i++) {
                Vertex v = vertices.get(i);
                g.setColor(new Color(255, 69, 0)); // Orange-red for vertices
                g.fillOval(v.x - 15, v.y - 15, 30, 30);
                g.setColor(Color.WHITE); // White for vertex index
                g.setFont(new java.awt.Font("Arial", java.awt.Font.BOLD, 12));
                g.drawString(String.valueOf(i), v.x - 5, v.y + 5);
            }

            // Draw selected vertex
            if (selectedVertex != null) {
                g.setColor(new Color(30, 144, 255)); // Dodger blue for selection
                g.drawOval(selectedVertex.x - 15, selectedVertex.y - 15, 30, 30);
            }
        }

        /**
         * Finds the vertex at the given coordinates.
         *
         * @param x X-coordinate of the click
         * @param y Y-coordinate of the click
         * @return Vertex at the coordinates, or null if none found
         */
        private Vertex getVertexAt(int x, int y) {
            for (Vertex v : vertices) {
                if (Math.sqrt(Math.pow(v.x - x, 2) + Math.pow(v.y - y, 2)) <= 15) {
                    return v;
                }
            }
            return null;
        }

        /**
         * Finds the edge at the given coordinates within a tolerance.
         *
         * @param x X-coordinate of the click
         * @param y Y-coordinate of the click
         * @return Edge at the coordinates, or null if none found
         */
        private Edge getEdgeAt(int x, int y) {
            final int TOLERANCE = 5; // Distance tolerance for edge detection
            
            for (Edge edge : edges) {
                // Calculate distance from point to line segment
                double distance = distanceFromPointToLineSegment(x, y, edge.u.x, edge.u.y, edge.v.x, edge.v.y);
                if (distance <= TOLERANCE) {
                    return edge;
                }
            }
            return null;
        }

        /**
         * Calculates the distance from a point to a line segment.
         */
        private double distanceFromPointToLineSegment(int px, int py, int x1, int y1, int x2, int y2) {
            double dx = x2 - x1;
            double dy = y2 - y1;
            
            if (dx == 0 && dy == 0) {
                // Line segment is actually a point
                return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
            }
            
            double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
            t = Math.max(0, Math.min(1, t));
            
            double projX = x1 + t * dx;
            double projY = y1 + t * dy;
            
            return Math.sqrt((px - projX) * (px - projX) + (py - projY) * (py - projY));
        }
    }

    /**
     * Opens a graph from a text file.
     * File format:
     * Line 1: Number of vertices
     * Line 2-n: Vertex coordinates (x y)
     * Line n+1: Number of edges
     * Line n+2-end: Edge data (vertex1_index vertex2_index weight)
     */
    private void openGraph() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setFileFilter(new FileNameExtensionFilter("Text Files (*.txt)", "txt"));
        fileChooser.setDialogTitle("Open Graph File");

        int result = fileChooser.showOpenDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) {
            File selectedFile = fileChooser.getSelectedFile();
            try (BufferedReader reader = new BufferedReader(new FileReader(selectedFile))) {
                // Clear current graph
                vertices.clear();
                edges.clear();
                graphPanel.clearHighlight();

                // Read number of vertices
                String line = reader.readLine();
                if (line == null) {
                    throw new IOException("Invalid file format: missing vertex count");
                }
                int vertexCount = Integer.parseInt(line.trim());

                // Read vertex coordinates
                for (int i = 0; i < vertexCount; i++) {
                    line = reader.readLine();
                    if (line == null) {
                        throw new IOException("Invalid file format: missing vertex data");
                    }
                    String[] coords = line.trim().split("\\s+");
                    if (coords.length != 2) {
                        throw new IOException("Invalid vertex format: " + line);
                    }
                    int x = Integer.parseInt(coords[0]);
                    int y = Integer.parseInt(coords[1]);
                    vertices.add(new Vertex(x, y));
                }

                // Read number of edges
                line = reader.readLine();
                if (line == null) {
                    throw new IOException("Invalid file format: missing edge count");
                }
                int edgeCount = Integer.parseInt(line.trim());

                // Read edge data
                for (int i = 0; i < edgeCount; i++) {
                    line = reader.readLine();
                    if (line == null) {
                        throw new IOException("Invalid file format: missing edge data");
                    }
                    String[] edgeData = line.trim().split("\\s+");
                    if (edgeData.length != 3) {
                        throw new IOException("Invalid edge format: " + line);
                    }
                    int u = Integer.parseInt(edgeData[0]);
                    int v = Integer.parseInt(edgeData[1]);
                    int weight = Integer.parseInt(edgeData[2]);

                    if (u < 0 || u >= vertices.size() || v < 0 || v >= vertices.size()) {
                        throw new IOException("Invalid vertex index in edge: " + line);
                    }

                    edges.add(new Edge(vertices.get(u), vertices.get(v), weight));
                }

                // Update display
                updateInfo();
                graphPanel.repaint();

                JOptionPane.showMessageDialog(this,
                        "Graph loaded successfully!\nVertices: " + vertices.size() + "\nEdges: " + edges.size(),
                        "Success", JOptionPane.INFORMATION_MESSAGE);

            } catch (IOException | NumberFormatException e) {
                JOptionPane.showMessageDialog(this,
                        "Error loading graph: " + e.getMessage(),
                        "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    }

    /**
     * Saves the current graph to a text file.
     * File format:
     * Line 1: Number of vertices
     * Line 2-n: Vertex coordinates (x y)
     * Line n+1: Number of edges
     * Line n+2-end: Edge data (vertex1_index vertex2_index weight)
     */
    private void saveGraph() {
        if (vertices.isEmpty()) {
            JOptionPane.showMessageDialog(this,
                    "No graph to save. Please create a graph first.",
                    "Warning", JOptionPane.WARNING_MESSAGE);
            return;
        }

        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setFileFilter(new FileNameExtensionFilter("Text Files (*.txt)", "txt"));
        fileChooser.setDialogTitle("Save Graph As");
        fileChooser.setSelectedFile(new File("graph.txt"));

        int result = fileChooser.showSaveDialog(this);
        if (result == JFileChooser.APPROVE_OPTION) {
            File selectedFile = fileChooser.getSelectedFile();

            // Ensure .txt extension
            if (!selectedFile.getName().toLowerCase().endsWith(".txt")) {
                selectedFile = new File(selectedFile.getAbsolutePath() + ".txt");
            }

            try (BufferedWriter writer = new BufferedWriter(new FileWriter(selectedFile))) {
                // Write number of vertices
                writer.write(vertices.size() + "\n");

                // Write vertex coordinates
                for (Vertex vertex : vertices) {
                    writer.write(vertex.x + " " + vertex.y + "\n");
                }

                // Write number of edges
                writer.write(edges.size() + "\n");

                // Write edge data
                for (Edge edge : edges) {
                    int uIndex = vertices.indexOf(edge.u);
                    int vIndex = vertices.indexOf(edge.v);
                    writer.write(uIndex + " " + vIndex + " " + edge.weight + "\n");
                }

                JOptionPane.showMessageDialog(this,
                        "Graph saved successfully to: " + selectedFile.getAbsolutePath(),
                        "Success", JOptionPane.INFORMATION_MESSAGE);

            } catch (IOException e) {
                JOptionPane.showMessageDialog(this,
                        "Error saving graph: " + e.getMessage(),
                        "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    }
}
