clear all;
ch = canChannel('PEAK-System', 'PCAN_USBBUS1'); % Utworzenie połączenia

configBusSpeed(ch, 1000000); % Ustawienie prędkości na 1 Mbps

start(ch); % Uruchomienie komunikacji

% Utworzenie struktury ramki CAN (Rozszerzone ID: 15555556, DLC: 3)
msg = canMessage(hex2dec('15555556'), true, 3);
disp('Rozpoczynam test obciaznicy');

% Przejście w tryb ZDALNY
msg.Data = [255, 0, 0];
transmit(ch, msg);
disp('Wlaczono tryb zdalny');
pause(2);

% Wiersze to kolejne kroki. Kolumny to stany Modułów [M1, M2, M3]
sekwencja = [
    1, 1, 1;  % Krok 1: Wymuszenie 5.5 ohm na wszystkich
    2, 2, 2;  % Krok 2: Wymuszenie 22 ohm na wszystkich
    3, 3, 3;  % Krok 3: Wymuszenie 88 ohm na wszystkich
    1, 2, 3;  % Krok 4: Test mieszany
    3, 1, 2   % Krok 5: Test mieszany (zamiana)
    ];

for i = 1:size(sekwencja, 1)
    msg.Data = sekwencja(i, :);
    transmit(ch, msg);
    fprintf('Krok %d: Wyslano konfiguracje [%d, %d, %d]\n', i, msg.Data(1), msg.Data(2), msg.Data(3));
    pause(4); 
end

% Powrót do trybu RECZNEGO
msg.Data = [0, 0, 0];
transmit(ch, msg);
disp('Koniec testu. Powrot do trybu manulanego.');

% Zamkniecie polaczenia z magistrala
stop(ch);