module drone.dronepathplanning_dfs {
    requires javafx.controls;
    requires javafx.fxml;


    opens drone.dronepathplanning_dfs to javafx.fxml;
    exports drone.dronepathplanning_dfs;
}