package drone.dronepathplanning_dfs;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/** Depth-First Search (DFS); single drone in a static 2D occupancy grid. */
public final class DronePathPlanning {
    private final Point start, finish;
    private final boolean success;
    private final double pathCost;
    private final int expandedNodes;
    private final long elapsedNanos;
    private final int steps;
    private final String terminationReason;

    // Frozen movement order: N, NE, E, SE, S, SW, W, NW.
    private static final int[][] N8 = {
            {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
            {1, 0}, {1, -1}, {0, -1}, {-1, -1}
    };

    public DronePathPlanning(int[][] environment, Drone drone, Point finish) {
        if (environment == null || environment.length == 0 || environment[0] == null
                || environment[0].length == 0 || drone == null || finish == null)
            throw new IllegalArgumentException("Grid, drone, and finish must be provided");
        this.start = new Point(drone.initialRowPosition, drone.initialColPosition);
        this.finish = finish;
        int rows = environment.length, cols = environment[0].length;
        if (!free(environment, start.row(), start.col())
                || !free(environment, finish.row(), finish.col()))
            throw new IllegalArgumentException("Start or finish is outside grid or blocked");
        long begin = System.nanoTime();

        // =====================================================================
        // BEGIN REPLACEABLE ALGORITHM: DEPTH-FIRST SEARCH (DFS)
        // Only this algorithm block needs replacement for subsequent papers.
        // 8-connected; orthogonal cost = 1, diagonal cost = sqrt(2).
        // Diagonal corner cutting is prohibited.
        // DFS uses an explicit stack, and does NOT guarantee a shortest path.
        // =====================================================================
        boolean[][] visited = new boolean[rows][cols];
        Point[][] parent = new Point[rows][cols];
        ArrayDeque<Frame> stack = new ArrayDeque<>();
        stack.push(new Frame(start.row(), start.col()));
        visited[start.row()][start.col()] = true;

        int expanded = 1; // Count each node upon first entry into the DFS search.
        boolean reached = start.equals(finish);
        while (!stack.isEmpty() && !reached) {
            Frame current = stack.peek();
            if (current.nextNeighbor >= N8.length) {
                stack.pop(); // Backtrack after all neighbors have been considered.
                continue;
            }
            int[] delta = N8[current.nextNeighbor++];
            int nr = current.row + delta[0];
            int nc = current.col + delta[1];
            if (!free(environment, nr, nc) || visited[nr][nc]) continue;
            if (delta[0] != 0 && delta[1] != 0
                    && (!free(environment, current.row, nc)
                    || !free(environment, nr, current.col))) continue;

            visited[nr][nc] = true;
            parent[nr][nc] = new Point(current.row, current.col);
            expanded++;
            stack.push(new Frame(nr, nc)); // Explore this branch before its siblings.
            reached = nr == finish.row() && nc == finish.col();
        }

        ArrayList<Point> resultPath = new ArrayList<>();
        double cost = Double.NaN;
        if (reached) {
            LinkedList<Point> chain = new LinkedList<>();
            Point cursor = finish;
            while (cursor != null) {
                chain.addFirst(cursor);
                if (cursor.equals(start)) break;
                cursor = parent[cursor.row()][cursor.col()];
            }
            resultPath.addAll(chain);
            cost = 0.0;
            for (int i = 1; i < resultPath.size(); i++) {
                Point a = resultPath.get(i - 1), b = resultPath.get(i);
                cost += a.row() != b.row() && a.col() != b.col()
                        ? Math.sqrt(2.0) : 1.0;
            }
        }
        // =====================================================================
        // END REPLACEABLE ALGORITHM: DEPTH-FIRST SEARCH (DFS)
        // =====================================================================

        elapsedNanos = System.nanoTime() - begin;
        success = reached;
        expandedNodes = expanded;
        pathCost = cost;
        terminationReason = reached ? "GOAL_REACHED" : "NO_PATH";
        drone.path = reached ? resultPath : new ArrayList<>(List.of(start));
        drone.frameIndex = 0;
        steps = drone.path.size() - 1;
    }

    private static boolean free(int[][] env, int r, int c) {
        return r >= 0 && r < env.length && c >= 0
                && c < env[0].length && env[r][c] != -1;
    }

    /** An iterative DFS frame holds the next unexamined neighbor. */
    private static final class Frame {
        final int row, col;
        int nextNeighbor;
        Frame(int row, int col) { this.row = row; this.col = col; }
    }

    // Frozen API used by CanvasView and MainView.
    public Point getStart() { return start; }
    public Point getFinish() { return finish; }
    public boolean isSuccess() { return success; }
    public double getPathCost() { return pathCost; }
    public int getExpandedNodes() { return expandedNodes; }
    public long getElapsedNanos() { return elapsedNanos; }
    public int getSteps() { return steps; }
    public String getTerminationReason() { return terminationReason; }
}
