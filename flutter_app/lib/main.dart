import 'package:flutter/material.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:provider/provider.dart';
import 'package:flutter_bloc/flutter_bloc.dart';

import 'package:india_delivery_robot_app/localization/app_localizations.dart';
import 'package:india_delivery_robot_app/localization/localization_delegate.dart';
import 'package:india_delivery_robot_app/providers/language_provider.dart';
import 'package:india_delivery_robot_app/providers/robot_provider.dart';
import 'package:india_delivery_robot_app/screens/home_screen.dart';
import 'package:india_delivery_robot_app/screens/login_screen.dart';
import 'package:india_delivery_robot_app/screens/splash_screen.dart';
import 'package:india_delivery_robot_app/theme/app_theme.dart';
import 'package:india_delivery_robot_app/utils/app_constants.dart';

void main() {
  runApp(const IndiaDeliveryRobotApp());
}

class IndiaDeliveryRobotApp extends StatelessWidget {
  const IndiaDeliveryRobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => LanguageProvider()),
        ChangeNotifierProvider(create: (_) => RobotProvider()),
      ],
      child: Consumer<LanguageProvider>(
        builder: (context, languageProvider, child) {
          return MaterialApp(
            title: '🇮🇳 Indian Delivery Robot',
            debugShowCheckedModeBanner: false,
            
            // Localization
            locale: languageProvider.locale,
            localizationsDelegates: const [
              AppLocalizationsDelegate(),
              GlobalMaterialLocalizations.delegate,
              GlobalWidgetsLocalizations.delegate,
              GlobalCupertinoLocalizations.delegate,
            ],
            supportedLocales: AppConstants.supportedLocales,
            
            // Theme
            theme: AppTheme.lightTheme,
            darkTheme: AppTheme.darkTheme,
            themeMode: ThemeMode.system,
            
            // Routes
            initialRoute: AppConstants.splashRoute,
            routes: {
              AppConstants.splashRoute: (context) => const SplashScreen(),
              AppConstants.loginRoute: (context) => const LoginScreen(),
              AppConstants.homeRoute: (context) => const HomeScreen(),
            },
          );
        },
      ),
    );
  }
}
