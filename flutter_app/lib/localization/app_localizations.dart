import 'package:flutter/material.dart';

class AppLocalizations {
  final Locale locale;

  AppLocalizations(this.locale);

  static AppLocalizations of(BuildContext context) {
    return Localizations.of<AppLocalizations>(context, AppLocalizations)!;
  }

  static const LocalizationsDelegate<AppLocalizations> delegate =
      AppLocalizationsDelegate();

  // Common strings
  String get appTitle => _localizedValues[locale.languageCode]!['appTitle']!;
  String get welcome => _localizedValues[locale.languageCode]!['welcome']!;
  String get login => _localizedValues[locale.languageCode]!['login']!;
  String get logout => _localizedValues[locale.languageCode]!['logout']!;
  String get home => _localizedValues[locale.languageCode]!['home']!;
  String get settings => _localizedValues[locale.languageCode]!['settings']!;
  String get language => _localizedValues[locale.languageCode]!['language']!;
  String get english => _localizedValues[locale.languageCode]!['english']!;
  String get hindi => _localizedValues[locale.languageCode]!['hindi']!;
  String get tamil => _localizedValues[locale.languageCode]!['tamil']!;
  String get telugu => _localizedValues[locale.languageCode]!['telugu']!;
  String get bengali => _localizedValues[locale.languageCode]!['bengali']!;
  
  // Robot control
  String get robotStatus => _localizedValues[locale.languageCode]!['robotStatus']!;
  String get autonomous => _localizedValues[locale.languageCode]!['autonomous']!;
  String get manual => _localizedValues[locale.languageCode]!['manual']!;
  String get teleop => _localizedValues[locale.languageCode]!['teleop']!;
  String get emergencyStop => _localizedValues[locale.languageCode]!['emergencyStop']!;
  String get startMission => _localizedValues[locale.languageCode]!['startMission']!;
  String get stopMission => _localizedValues[locale.languageCode]!['stopMission']!;
  String get pauseMission => _localizedValues[locale.languageCode]!['pauseMission']!;
  
  // Navigation
  String get navigateTo => _localizedValues[locale.languageCode]!['navigateTo']!;
  String get currentLocation => _localizedValues[locale.languageCode]!['currentLocation']!;
  String get destination => _localizedValues[locale.languageCode]!['destination']!;
  String get route => _localizedValues[locale.languageCode]!['route']!;
  String get distance => _localizedValues[locale.languageCode]!['distance']!;
  String get estimatedTime => _localizedValues[locale.languageCode]!['estimatedTime']!;
  
  // Delivery
  String get delivery => _localizedValues[locale.languageCode]!['delivery']!;
  String get newDelivery => _localizedValues[locale.languageCode]!['newDelivery']!;
  String get customerName => _localizedValues[locale.languageCode]!['customerName']!;
  String get customerPhone => _localizedValues[locale.languageCode]!['customerPhone']!;
  String get deliveryAddress => _localizedValues[locale.languageCode]!['deliveryAddress']!;
  String get orderItems => _localizedValues[locale.languageCode]!['orderItems']!;
  String get specialInstructions => _localizedValues[locale.languageCode]!['specialInstructions']!;
  String get deliveryFee => _localizedValues[locale.languageCode]!['deliveryFee']!;
  String get priority => _localizedValues[locale.languageCode]!['priority']!;
  String get high => _localizedValues[locale.languageCode]!['high']!;
  String get medium => _localizedValues[locale.languageCode]!['medium']!;
  String get low => _localizedValues[locale.languageCode]!['low']!;
  
  // Status
  String get pending => _localizedValues[locale.languageCode]!['pending']!;
  String get inProgress => _localizedValues[locale.languageCode]!['inProgress']!;
  String get completed => _localizedValues[locale.languageCode]!['completed']!;
  String get failed => _localizedValues[locale.languageCode]!['failed']!;
  String get cancelled => _localizedValues[locale.languageCode]!['cancelled']!;
  
  // Sensors and Environment
  String get batteryLevel => _localizedValues[locale.languageCode]!['batteryLevel']!;
  String get obstacleDetection => _localizedValues[locale.languageCode]!['obstacleDetection']!;
  String get pedestrianCount => _localizedValues[locale.languageCode]!['pedestrianCount']!;
  String get animalCount => _localizedValues[locale.languageCode]!['animalCount']!;
  String get weatherCondition => _localizedValues[locale.languageCode]!['weatherCondition']!;
  String get clear => _localizedValues[locale.languageCode]!['clear']!;
  String get rain => _localizedValues[locale.languageCode]!['rain']!;
  String get fog => _localizedValues[locale.languageCode]!['fog']!;
  String get dust => _localizedValues[locale.languageCode]!['dust']!;
  
  // Messages
  String get missionStarted => _localizedValues[locale.languageCode]!['missionStarted']!;
  String get missionCompleted => _localizedValues[locale.languageCode]!['missionCompleted']!;
  String get missionFailed => _localizedValues[locale.languageCode]!['missionFailed']!;
  String get robotConnected => _localizedValues[locale.languageCode]!['robotConnected']!;
  String get robotDisconnected => _localizedValues[locale.languageCode]!['robotDisconnected']!;
  String get lowBattery => _localizedValues[locale.languageCode]!['lowBattery']!;
  String get obstacleDetected => _localizedValues[locale.languageCode]!['obstacleDetected']!;
  
  // Buttons
  String get confirm => _localizedValues[locale.languageCode]!['confirm']!;
  String get cancel => _localizedValues[locale.languageCode]!['cancel']!;
  String get save => _localizedValues[locale.languageCode]!['save']!;
  String get edit => _localizedValues[locale.languageCode]!['edit']!;
  String get delete => _localizedValues[locale.languageCode]!['delete']!;
  String get retry => _localizedValues[locale.languageCode]!['retry']!;
  String get refresh => _localizedValues[locale.languageCode]!['refresh']!;
  
  // Errors
  String get error => _localizedValues[locale.languageCode]!['error']!;
  String get networkError => _localizedValues[locale.languageCode]!['networkError']!;
  String get connectionError => _localizedValues[locale.languageCode]!['connectionError']!;
  String get invalidInput => _localizedValues[locale.languageCode]!['invalidInput']!;
  String get permissionDenied => _localizedValues[locale.languageCode]!['permissionDenied']!;

  static final Map<String, Map<String, String>> _localizedValues = {
    'en': {
      'appTitle': '🇮🇳 Indian Delivery Robot',
      'welcome': 'Welcome',
      'login': 'Login',
      'logout': 'Logout',
      'home': 'Home',
      'settings': 'Settings',
      'language': 'Language',
      'english': 'English',
      'hindi': 'हिंदी',
      'tamil': 'தமிழ்',
      'telugu': 'తెలుగు',
      'bengali': 'বাংলা',
      
      'robotStatus': 'Robot Status',
      'autonomous': 'Autonomous',
      'manual': 'Manual',
      'teleop': 'Teleoperation',
      'emergencyStop': 'Emergency Stop',
      'startMission': 'Start Mission',
      'stopMission': 'Stop Mission',
      'pauseMission': 'Pause Mission',
      
      'navigateTo': 'Navigate To',
      'currentLocation': 'Current Location',
      'destination': 'Destination',
      'route': 'Route',
      'distance': 'Distance',
      'estimatedTime': 'Estimated Time',
      
      'delivery': 'Delivery',
      'newDelivery': 'New Delivery',
      'customerName': 'Customer Name',
      'customerPhone': 'Customer Phone',
      'deliveryAddress': 'Delivery Address',
      'orderItems': 'Order Items',
      'specialInstructions': 'Special Instructions',
      'deliveryFee': 'Delivery Fee',
      'priority': 'Priority',
      'high': 'High',
      'medium': 'Medium',
      'low': 'Low',
      
      'pending': 'Pending',
      'inProgress': 'In Progress',
      'completed': 'Completed',
      'failed': 'Failed',
      'cancelled': 'Cancelled',
      
      'batteryLevel': 'Battery Level',
      'obstacleDetection': 'Obstacle Detection',
      'pedestrianCount': 'Pedestrians',
      'animalCount': 'Animals',
      'weatherCondition': 'Weather',
      'clear': 'Clear',
      'rain': 'Rain',
      'fog': 'Fog',
      'dust': 'Dust',
      
      'missionStarted': 'Mission Started',
      'missionCompleted': 'Mission Completed',
      'missionFailed': 'Mission Failed',
      'robotConnected': 'Robot Connected',
      'robotDisconnected': 'Robot Disconnected',
      'lowBattery': 'Low Battery Warning',
      'obstacleDetected': 'Obstacle Detected',
      
      'confirm': 'Confirm',
      'cancel': 'Cancel',
      'save': 'Save',
      'edit': 'Edit',
      'delete': 'Delete',
      'retry': 'Retry',
      'refresh': 'Refresh',
      
      'error': 'Error',
      'networkError': 'Network Error',
      'connectionError': 'Connection Error',
      'invalidInput': 'Invalid Input',
      'permissionDenied': 'Permission Denied',
    },
    'hi': {
      'appTitle': '🇮🇳 भारतीय डिलीवरी रोबोट',
      'welcome': 'स्वागत है',
      'login': 'लॉगिन',
      'logout': 'लॉगआउट',
      'home': 'होम',
      'settings': 'सेटिंग्स',
      'language': 'भाषा',
      'english': 'English',
      'hindi': 'हिंदी',
      'tamil': 'தமிழ்',
      'telugu': 'తెలుగు',
      'bengali': 'বাংলা',
      
      'robotStatus': 'रोबोट स्थिति',
      'autonomous': 'स्वायत्त',
      'manual': 'मैनुअल',
      'teleop': 'टेलीऑपरेशन',
      'emergencyStop': 'आपातकालीन रोक',
      'startMission': 'मिशन शुरू करें',
      'stopMission': 'मिशन रोकें',
      'pauseMission': 'मिशन रोकें',
      
      'navigateTo': 'नेविगेट करें',
      'currentLocation': 'वर्तमान स्थान',
      'destination': 'गंतव्य',
      'route': 'मार्ग',
      'distance': 'दूरी',
      'estimatedTime': 'अनुमानित समय',
      
      'delivery': 'डिलीवरी',
      'newDelivery': 'नई डिलीवरी',
      'customerName': 'ग्राहक का नाम',
      'customerPhone': 'ग्राहक का फोन',
      'deliveryAddress': 'डिलीवरी पता',
      'orderItems': 'ऑर्डर आइटम',
      'specialInstructions': 'विशेष निर्देश',
      'deliveryFee': 'डिलीवरी शुल्क',
      'priority': 'प्राथमिकता',
      'high': 'उच्च',
      'medium': 'मध्यम',
      'low': 'कम',
      
      'pending': 'लंबित',
      'inProgress': 'प्रगति में',
      'completed': 'पूर्ण',
      'failed': 'असफल',
      'cancelled': 'रद्द',
      
      'batteryLevel': 'बैटरी स्तर',
      'obstacleDetection': 'बाधा का पता लगाना',
      'pedestrianCount': 'पैदल यात्री',
      'animalCount': 'जानवर',
      'weatherCondition': 'मौसम',
      'clear': 'साफ',
      'rain': 'बारिश',
      'fog': 'कोहरा',
      'dust': 'धूल',
      
      'missionStarted': 'मिशन शुरू हुआ',
      'missionCompleted': 'मिशन पूर्ण हुआ',
      'missionFailed': 'मिशन असफल',
      'robotConnected': 'रोबोट जुड़ा',
      'robotDisconnected': 'रोबोट डिस्कनेक्ट',
      'lowBattery': 'कम बैटरी चेतावनी',
      'obstacleDetected': 'बाधा का पता चला',
      
      'confirm': 'पुष्टि करें',
      'cancel': 'रद्द करें',
      'save': 'सहेजें',
      'edit': 'संपादित करें',
      'delete': 'हटाएं',
      'retry': 'पुनः प्रयास',
      'refresh': 'रिफ्रेश',
      
      'error': 'त्रुटि',
      'networkError': 'नेटवर्क त्रुटि',
      'connectionError': 'कनेक्शन त्रुटि',
      'invalidInput': 'अमान्य इनपुट',
      'permissionDenied': 'अनुमति अस्वीकृत',
    },
    'ta': {
      'appTitle': '🇮🇳 இந்திய டெலிவரி ரோபோட்',
      'welcome': 'வரவேற்கிறோம்',
      'login': 'உள்நுழைவு',
      'logout': 'வெளியேறு',
      'home': 'வீடு',
      'settings': 'அமைப்புகள்',
      'language': 'மொழி',
      'english': 'English',
      'hindi': 'हिंदी',
      'tamil': 'தமிழ்',
      'telugu': 'తెలుగు',
      'bengali': 'বাংলা',
      
      'robotStatus': 'ரோபோட் நிலை',
      'autonomous': 'தன்னாட்சி',
      'manual': 'கைமுறை',
      'teleop': 'தொலை இயக்கம்',
      'emergencyStop': 'அவசர நிறுத்தம்',
      'startMission': 'பணி தொடங்கு',
      'stopMission': 'பணி நிறுத்து',
      'pauseMission': 'பணி இடைநிறுத்து',
      
      'navigateTo': 'வழிகாட்டு',
      'currentLocation': 'தற்போதைய இடம்',
      'destination': 'இலக்கு',
      'route': 'வழி',
      'distance': 'தூரம்',
      'estimatedTime': 'மதிப்பிடப்பட்ட நேரம்',
      
      'delivery': 'டெலிவரி',
      'newDelivery': 'புதிய டெலிவரி',
      'customerName': 'வாடிக்கையாளர் பெயர்',
      'customerPhone': 'வாடிக்கையாளர் தொலைபேசி',
      'deliveryAddress': 'டெலிவரி முகவரி',
      'orderItems': 'ஆர்டர் உருப்படிகள்',
      'specialInstructions': 'சிறப்பு வழிமுறைகள்',
      'deliveryFee': 'டெலிவரி கட்டணம்',
      'priority': 'முன்னுரிமை',
      'high': 'உயர்',
      'medium': 'நடுத்தர',
      'low': 'குறைந்த',
      
      'pending': 'நிலுவையில்',
      'inProgress': 'முன்னேற்றத்தில்',
      'completed': 'முடிந்தது',
      'failed': 'தோல்வி',
      'cancelled': 'ரத்து',
      
      'batteryLevel': 'பேட்டரி நிலை',
      'obstacleDetection': 'தடையை கண்டறிதல்',
      'pedestrianCount': 'பாதசாரிகள்',
      'animalCount': 'விலங்குகள்',
      'weatherCondition': 'வானிலை',
      'clear': 'தெளிவு',
      'rain': 'மழை',
      'fog': 'மூடுபனி',
      'dust': 'தூசி',
      
      'missionStarted': 'பணி தொடங்கியது',
      'missionCompleted': 'பணி முடிந்தது',
      'missionFailed': 'பணி தோல்வி',
      'robotConnected': 'ரோபோட் இணைக்கப்பட்டது',
      'robotDisconnected': 'ரோபோட் துண்டிக்கப்பட்டது',
      'lowBattery': 'குறைந்த பேட்டரி எச்சரிக்கை',
      'obstacleDetected': 'தடை கண்டறியப்பட்டது',
      
      'confirm': 'உறுதிப்படுத்து',
      'cancel': 'ரத்து செய்',
      'save': 'சேமி',
      'edit': 'திருத்து',
      'delete': 'நீக்கு',
      'retry': 'மீண்டும் முயற்சி',
      'refresh': 'புதுப்பி',
      
      'error': 'பிழை',
      'networkError': 'நெட்வொர்க் பிழை',
      'connectionError': 'இணைப்பு பிழை',
      'invalidInput': 'தவறான உள்ளீடு',
      'permissionDenied': 'அனுமதி மறுக்கப்பட்டது',
    },
  };
}
