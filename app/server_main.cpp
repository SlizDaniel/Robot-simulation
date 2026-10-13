#include <httplib.h>
#include <iostream>

int main() {
    httplib::Server server;

    server.Get("/api/health",
        [](const httplib::Request&,
           httplib::Response& response) {

            response.set_content(
                R"({"status":"ok","backend":"cpp"})",
                "application/json"
            );
        }
    );

    std::cout << "Server: http://127.0.0.1:8080\n";

    if (!server.set_mount_point("/", "./frontend")) {
    std::cerr << "Cannot find frontend directory\n";
    return 1;
    }
    
    if (!server.listen("127.0.0.1", 8080)) {
        std::cerr << "Failed to start server\n";
        return 1;
    }

    return 0;
}