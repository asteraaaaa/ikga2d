module com.csc4702.ikga2d {
    requires javafx.controls;
    requires javafx.fxml;


    opens com.csc4702.ikga2d to javafx.fxml;
    exports com.csc4702.ikga2d;
}