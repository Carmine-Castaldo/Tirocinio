#include <iostream>
#include <thread>
#include <chrono>

#include "RemoteAPIClient.h"
#include "RemoteAPIObjects.h"

int main() {
    std::cout << "=== TEST CONNESSIONE COPPELIASIM ===" << std::endl;

    try {
        RemoteAPIClient client("127.0.0.1", 23000);
        RemoteAPIObjects simObj(&client);

        std::cout << "✅ Connessione stabilita con successo a CoppeliaSim!" << std::endl;

    
        double simTime = simObj.sim().getSimulationTime();
        std::cout << "Tempo di simulazione corrente: " << simTime << " s" << std::endl;

        
        int64_t robotHandle = simObj.sim().getObjectHandle("/PioneerP3DX");
        if (robotHandle != -1) {
            std::cout << "Robot /PioneerP3DX trovato (Handle: " << robotHandle << ")" << std::endl;
        } else {
            std::cout << " Robot /PioneerP3DX non trovato nella scena." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Errore di connessione: " << e.what() << std::endl;
        std::cerr << "Assicurati che CoppeliaSim sia aperto e la simulazione sia in PLAY." << std::endl;
        return 1;
    }

    return 0;
}